use std::f32::consts::TAU;
use std::sync::Arc;

use mdp::StateSpace;

use crate::RobotVector;

#[derive(Default, Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub struct DiscreteState {
    pub position: RobotVector<i32>,
}

#[derive(Default, Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub struct DiscreteAction {
    pub velocity: RobotVector<i32>,
}

#[derive(Default, Debug)]
#[derive(PartialOrd, PartialEq)]
#[derive(Clone)]
pub struct RobotStateSpace {
    pub length: u16,
    pub width: u16,
    pub max_pos_r: u16,
    pub turn_rate: u16,
    pub max_vel_r: u16,
    pub max_vel_p: u16,
    pub noise: f32,
    pub goal: RobotVector<i32>,
}

impl RobotStateSpace {
    pub(crate) fn validate_state(&self, state: &DiscreteState) -> bool {
        state.position.x >= 0 && state.position.x < self.width as i32
            && state.position.y >= 0 && state.position.y < self.length as i32
    }

    pub(crate) fn apply_state(&self, state: &DiscreteState, action: &DiscreteAction) -> DiscreteState {
        DiscreteState {
            position: RobotVector {
                x: state.position.x + action.velocity.x,
                y: state.position.y + action.velocity.y,
                r: (state.position.r + action.velocity.r) % self.max_pos_r as i32,
            }
        }
    }
}

impl StateSpace<DiscreteState, DiscreteAction> for RobotStateSpace {
    fn nonterminal_states(&self) -> Arc<Vec<DiscreteState>> {
        todo!()
    }

    fn terminal(&self, state: &DiscreteState) -> bool {
        let theta = state.position.r == self.goal.r && self.goal.r >= 0;
        let x = state.position.x == self.goal.x;
        let y = state.position.y == self.goal.y;
        x && y && theta
    }

    fn actions(&self, state: &DiscreteState) -> Vec<DiscreteAction> {
        let mut actions = Vec::new();

        for speed in (-(self.max_vel_p as i32))..(self.max_vel_p as i32) {
            for omega in (-(self.max_vel_r as i32))..(self.max_vel_r as i32) {
                let theta = (state.position.r as f32 / self.max_pos_r as f32) * TAU;

                let a = DiscreteAction {
                    velocity: RobotVector {
                        x: (speed as f32 * theta.cos()).round() as i32,
                        y: (speed as f32 * theta.sin()).round() as i32,
                        r: omega,
                    }
                };

                let new_state = self.apply_state(&state, &a);

                if self.validate_state(&new_state) {
                    actions.push(a)
                }
            }
        }

        actions
    }

    fn q_states(&self, state: &DiscreteState, action: &DiscreteAction) -> Vec<(f32, DiscreteState)> {
        const NOISE_DIST: i32 = 2;

        let mut q_states = Vec::new();

        for dx in -NOISE_DIST..=NOISE_DIST {
            for dy in -NOISE_DIST..NOISE_DIST {
                for dr in -NOISE_DIST..NOISE_DIST {
                    let mut new_state = DiscreteState {
                        position: state.position + RobotVector { x: dx, y: dy, r: dr }
                    };

                    new_state.position.r %= self.max_pos_r as i32;

                    if self.validate_state(&new_state) {
                        q_states.push((self.noise / (2 * NOISE_DIST + 1) as f32, new_state))
                    }
                }
            }
        }

        q_states.push((1.0 - self.noise, self.apply_state(&state, &action)));

        q_states
    }
}