//!
//! # Units
//! Units are Kg/m/s/rads

use std::f32::consts::TAU;
use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

use mdp::{Action, State, TransitionTable};
#[cfg(feature = "sim")]
use mdp::{RewardTable, StateSpace, TerminalTable};
use mdp::value_iteration::{ForecastTable, ForecastTableWriteView, ForecastTableReadView};

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
pub struct NavMap {
    pub max_forward_vel: f32,
    pub max_angular_vel: f32,
    pub vel_res: VectorResolution,
    pub pos_res: VectorResolution,
    pub map_length: f32,
    pub map_width: f32,
}

#[cfg(feature = "sim")]
#[derive(Default, Debug, Clone, PartialOrd, PartialEq)]
pub struct SimNavRewardTable {
    pub state_space: NavMap,
}

#[cfg(feature = "sim")]
#[derive(Default, Debug, Clone, PartialOrd, PartialEq)]
pub struct SimNavTermTable {
    pub state_space: NavMap,
}

#[cfg(feature = "sim")]
#[derive(Debug, Clone, PartialOrd, PartialEq)]
pub struct SimNavStateSpace {
    states: Arc<Vec<NavState>>
}

#[cfg(feature = "sim")]
impl SimNavStateSpace {
    pub fn new(map: NavMap) -> Self {
        let capacity = (map.map_length / map.pos_res.forward_step)
            * (map.map_width / map.pos_res.forward_step)
            * (TAU / map.pos_res.angular_step)
            * (map.max_forward_vel / map.vel_res.forward_step).powi(2)
            * (map.max_angular_vel / map.vel_res.angular_step);

        dbg!(capacity);

        let mut s = Vec::with_capacity(capacity as usize);

        let mut xp = 0.0;
        while xp <= map.map_width {
            let mut yp = 0.0;
            while yp <= map.map_length {
                let mut xv = 0.0;
                while xv <= map.max_forward_vel {
                    let mut yv = 0.0;
                    while yv <= map.max_forward_vel {
                        let mut tp = 0.0;
                        while tp <= TAU {
                            let mut tv = 0.0;
                            while tv <= map.max_angular_vel {
                                s.push(NavState {
                                    position: SpatialVector {
                                        x: xp,
                                        y: yp,
                                        theta: tp,
                                    },
                                    velocity: SpatialVector {
                                        x: xv,
                                        y: yv,
                                        theta: tv,
                                    },
                                });
                                tv += map.pos_res.angular_step;
                            }
                            tp += map.pos_res.angular_step;
                        }
                        yv += map.vel_res.forward_step;
                    }
                    xv += map.vel_res.forward_step;
                }
                yp += map.pos_res.forward_step;
            }
            xp += map.pos_res.forward_step;
        }

        let states = Arc::new(s);

        Self { states }
    }
}

#[cfg(feature = "sim")]
impl StateSpace<NavState> for SimNavStateSpace {
    fn nonterminal_states(&self) -> Arc<Vec<NavState>> {
        self.states.clone()
    }
}

#[cfg(feature = "sim")]
impl RewardTable<NavState, NavAction> for SimNavRewardTable {
    fn reward(&self, _: &NavState, end: &NavState, _: &NavAction) -> f32 {
        if (end.position.x - 5.0).abs() < self.state_space.pos_res.forward_step
            && (end.position.y - 5.0).abs() < self.state_space.pos_res.forward_step
        {
            1.0
        } else {
            -(end.position.x - 5.0).abs().sqrt() - (end.position.y - 5.0).abs().sqrt()
        }
    }
}

#[cfg(feature = "sim")]
impl TerminalTable<NavState> for SimNavTermTable {
    fn terminal(&self, state: &NavState) -> bool {
        if (state.position.x - 5.0).abs() < self.state_space.pos_res.forward_step
            && (state.position.y - 5.0).abs() < self.state_space.pos_res.forward_step
        {
            true
        } else {
            false
        }
    }
}


#[derive(Default, Debug, Clone, PartialOrd, PartialEq)]
pub struct NavStateTransitionTable {
    pub params: RobotParams,

    pub max_accel: f32,
    pub accel_step: f32,

    pub time_step: f32,

    pub state_space: NavMap,
    pub action_space: Vec<NavAction>,
}

impl NavStateTransitionTable {
    fn snap_to_nearest(&self, state: &mut NavState) {
        state.velocity.snap(self.state_space.vel_res);
        state.position.snap(self.state_space.pos_res)
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
            .filter(|(_, s)| s.position.x <= self.state_space.map_width)
            .filter(|(_, s)| s.position.x >= 0.0)
            .filter(|(_, s)| s.position.y <= self.state_space.map_length)
            .filter(|(_, s)| s.position.y >= 0.0)
            .filter(|(_, s)| s.velocity.x.abs() <= self.state_space.max_forward_vel)
            .filter(|(_, s)| s.velocity.y.abs() <= self.state_space.max_forward_vel)
            .filter(|(_, s)| s.velocity.theta.abs() <= self.state_space.max_angular_vel)
            .filter(|(_, s)| s.velocity.theta.abs() <= self.state_space.max_angular_vel)
            .collect();

        output.iter_mut()
            .for_each(|(_, s)| self.snap_to_nearest(s));

        output
    }
}

#[derive(Clone)]
pub struct NavForecastTable {
    pub inner: Arc<RwLock<Vec<f32>>>
}

pub struct NavForecastTableReadView<'a> {
    pub inner: RwLockReadGuard<'a, Vec<f32>>
}

pub struct NavForecastTableWriteView<'a> {
    pub inner: RwLockWriteGuard<'a, Vec<f32>>
}

impl<'a> ForecastTableReadView<NavState> for NavForecastTableReadView<'a> {
    fn read_forecast(&self, state: &NavState) -> f32 {
        todo!()
    }
}

impl<'a> ForecastTableWriteView<NavState> for NavForecastTableWriteView<'a> {
    fn write_forecast(&mut self, state: &NavState, value: f32) {
        todo!()
    }
}

impl<'a> ForecastTable<'a, NavState> for NavForecastTable {
    type ReadView = NavForecastTableReadView<'a>;
    type WriteView = NavForecastTableWriteView<'a>;

    fn read(&'a self) -> Self::ReadView {
        NavForecastTableReadView {
            inner: self.inner.read().unwrap()
        }
    }

    fn write(&'a self) -> Self::WriteView {
        NavForecastTableWriteView {
            inner: self.inner.write().unwrap()
        }
    }
}

impl State for NavState {}

impl Action for NavAction {}

