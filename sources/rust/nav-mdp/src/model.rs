//!
//! # Units
//! Units are Kg/m/s/rads

use mdp::{Action, State, TransitionTable};
use std::f32::consts::TAU;

#[derive(Default, Debug, Copy, Clone, PartialOrd, PartialEq)]
pub struct RobotParams {
    pub forward_friction: f32,
    pub angular_friction: f32,
}

#[derive(Default, Debug, Copy, Clone, PartialOrd, PartialEq)]
pub struct SpatialVector {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[derive(Default, Debug, Copy, Clone, PartialOrd, PartialEq)]
pub struct VectorResolution {
    pub forward_step: f32,
    pub angular_step: f32,
}

impl SpatialVector {
    fn snap(&mut self, res: VectorResolution) {
        self.x = (self.x / res.forward_step).round() * res.forward_step;
        self.y = (self.y / res.forward_step).round() * res.forward_step;
        self.theta = (self.theta / res.angular_step).round() * res.angular_step;
    }
}

#[derive(Default, Debug, Copy, Clone, PartialOrd, PartialEq)]
pub struct NavAction {
    pub forward_accel: f32,
    pub angular_accel: f32,
}

#[derive(Default, Debug, Clone, PartialOrd, PartialEq)]
pub struct NavState {
    pub position: SpatialVector,
    pub velocity: SpatialVector,
}

impl NavState {
    pub fn apply_action(&self, action: &NavAction, params: &RobotParams, time_step: f32) -> Self {
        let mut new_vel = self.velocity;
        let mut new_pos = self.position;

        let net_accel = SpatialVector {
            x: self.position.theta.cos() * action.forward_accel
                + self.velocity.x.signum() * params.forward_friction,
            y: self.position.theta.sin() * action.forward_accel
                + self.velocity.y.signum() * params.forward_friction,
            theta: action.angular_accel + self.velocity.theta.signum() * params.angular_friction,
        };

        new_vel.x += net_accel.x * time_step;
        new_vel.y += net_accel.y * time_step;
        new_vel.theta += net_accel.theta * time_step;

        new_pos.x += self.velocity.x * time_step;
        new_pos.y += self.velocity.y * time_step;
        new_pos.theta += self.velocity.theta * time_step;

        if new_pos.theta >= TAU {
            while new_pos.theta >= TAU {
                new_pos.theta -= TAU;
            }
        } else if new_pos.theta <= TAU {
            while new_pos.theta <= TAU {
                new_pos.theta += TAU;
            }
        }

        Self {
            position: new_pos,
            velocity: new_vel,
        }
    }
}

#[derive(Default, Debug, Clone, PartialOrd, PartialEq)]
pub struct NavStateTransitionTable {
    pub params: RobotParams,

    pub max_accel: f32,
    pub accel_step: f32,

    pub max_forward_vel: f32,
    pub max_angular_vel: f32,

    pub vel_res: VectorResolution,
    pub pos_res: VectorResolution,

    pub time_step: f32,

    pub map_length: f32,
    pub map_width: f32,

    pub action_space: Vec<NavAction>,
}

impl NavStateTransitionTable {
    fn snap_to_nearest(&self, state: &mut NavState) {
        state.velocity.snap(self.vel_res);
        state.position.snap(self.pos_res)
    }

    pub fn init_action_space(&mut self) {
        let cache = &mut self.action_space;

        if cache.is_empty() {
            let mut pos_accel = -self.max_accel;

            while pos_accel <= self.max_accel {
                let mut ang_accel = -self.max_accel;

                while ang_accel <= self.max_accel {
                    if (pos_accel + ang_accel).abs() <= self.max_accel {
                        cache.push(NavAction { forward_accel: pos_accel, angular_accel: ang_accel })
                    }

                    ang_accel += self.accel_step;
                }

                pos_accel += self.accel_step;
            }
        }
    }
}

impl TransitionTable<NavState, NavAction> for NavStateTransitionTable {
    fn list_transitions(&self, state: &NavState) -> Vec<(NavAction, NavState)> {
        let mut output: Vec<(NavAction, NavState)> = self.action_space.iter()
            .map(|a| (*a, state.apply_action(a, &self.params, self.time_step)))
            .filter(|(_,s)| s.position.x <= self.map_width)
            .filter(|(_,s)| s.position.x >= 0.0)
            .filter(|(_,s)| s.position.y <= self.map_length)
            .filter(|(_,s)| s.position.y >= 0.0)
            .filter(|(_,s)| s.velocity.x.abs() <= self.max_forward_vel)
            .filter(|(_,s)| s.velocity.y.abs() <= self.max_forward_vel)
            .filter(|(_,s)| s.velocity.theta.abs() <= self.max_angular_vel)
            .filter(|(_,s)| s.velocity.theta.abs() <= self.max_angular_vel)
            .collect();

        output.iter_mut()
            .for_each(|(_, s)| self.snap_to_nearest(s));

        output
    }
}

impl State for NavState {}

impl Action for NavAction {}