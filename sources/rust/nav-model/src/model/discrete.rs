use std::f32::consts::TAU;
use std::sync::Arc;

use nav_algo::{RewardTable, StateSpace};
use nav_algo::mdp::value_iteration::{ForecastTable, ForecastTableReadView};

use crate::{PerlinTable, RobotForecastTable, RobotVector};

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
    pub ang_res: u16,
    pub max_omega: u16,
    pub min_speed: u16,
    pub max_speed: u16,
    pub noise: f32,
    pub goal: RobotVector<i32>,
}

impl RobotStateSpace {
    pub(crate) fn validate_state(&self, state: &DiscreteState) -> bool {
        state.position.r >= 0
            && state.position.x >= 0
            && state.position.x < self.width as i32
            && state.position.y >= 0
            && state.position.y < self.length as i32
    }

    pub(crate) fn apply_state(&self, state: &DiscreteState, action: &DiscreteAction) -> DiscreteState {
        DiscreteState {
            position: RobotVector {
                x: state.position.x + action.velocity.x,
                y: state.position.y + action.velocity.y,
                r: (state.position.r + action.velocity.r + self.ang_res as i32) % self.ang_res as i32,
            }
        }
    }
}

impl StateSpace<DiscreteState, DiscreteAction> for RobotStateSpace {
    fn nonterminal_states(&self) -> Arc<Vec<DiscreteState>> {
        let mut states = Vec::new();

        for x in 0..self.width {
            for y in 0..self.length {
                for r in 0..self.ang_res {
                    let v = RobotVector {
                        x: x as i32,
                        y: y as i32,
                        r: r as i32,
                    };

                    let s = DiscreteState { position: v };

                    if !self.terminal(&s) {
                        states.push(s);
                    }
                }
            }
        }

        Arc::new(states)
    }

    fn terminal(&self, state: &DiscreteState) -> bool {
        state.position.x == self.goal.x
            && state.position.y == self.goal.y
            && (self.goal.r < 0 || state.position.r == self.goal.r)
    }

    fn actions(&self, state: &DiscreteState) -> Vec<DiscreteAction> {
        let mut actions = Vec::new();

        for speed in (-(self.max_speed as i32)..-(self.min_speed as i32))
            .chain((self.min_speed as i32)..(self.max_speed as i32))
        {
            for omega in (-(self.max_omega as i32))..(self.max_omega as i32) {
                let theta = (state.position.r as f32 / self.ang_res as f32) * TAU;

                let a = DiscreteAction {
                    velocity: RobotVector {
                        x: (speed as f32 * theta.cos()).round() as i32,
                        y: (speed as f32 * theta.sin()).round() as i32,
                        r: (omega + self.ang_res as i32) % self.ang_res as i32,
                    }
                };

                let new_state = self.apply_state(&state, &a);

                if self.validate_state(&new_state) {
                    actions.push(a)
                }
            }
        }

        actions.sort();
        actions.dedup();

        actions
    }

    fn q_states(&self, state: &DiscreteState, action: &DiscreteAction) -> Vec<(f32, DiscreteState)> {
        const NOISE_DIST: i32 = 1;

        let mut q_states = Vec::with_capacity(((NOISE_DIST as usize * 2) + 1).pow(3));

        let mut i = 0;

        for dx in -NOISE_DIST..=NOISE_DIST {
            for dy in -NOISE_DIST..=NOISE_DIST {
                for dr in -NOISE_DIST..=NOISE_DIST {
                    i += 1;

                    let mut new_state = DiscreteState {
                        position: state.position + RobotVector { x: dx, y: dy, r: dr }
                    };

                    new_state.position.r += self.ang_res as i32;
                    new_state.position.r %= self.ang_res as i32;

                    if self.validate_state(&new_state) {
                        q_states.push((self.noise, new_state));
                    }
                }
            }
        }

        for (p, _) in q_states.iter_mut() {
            *p /= i as f32;
        }

        q_states.push((1.0 - self.noise, self.apply_state(&state, &action)));

        q_states
    }
}

pub fn get_path(
    space: &RobotStateSpace,
    start: &DiscreteState,
    rewards: &PerlinTable,
    forecasts: &RobotForecastTable,
)
    -> Vec<DiscreteState> {
    let mut current = *start;
    let mut path = Vec::new();
    let mut count = 0;

    let forecast_read_guard = forecasts.read();

    while !space.terminal(&current) {
        let actions = space.actions(&current);

        let mut states: Vec<_> = actions.iter()
            .map(|a| {
                let mut q_states = space.q_states(&current, a);

                let mut value = 0.0;

                for (p, s) in &q_states {
                    let v = rewards.reward(s) + forecast_read_guard.read_forecast(s);
                    value += *p * v;
                }

                (value, q_states.pop().unwrap().1)
            })
            .collect();

        let (mut best_value, mut best) = (f32::MIN, DiscreteState::default());

        while let Some((value, state)) = states.pop() {
            if value > best_value {
                best_value = value;
                best = state;
            }
        }

        current = best;

        count += 1;

        if path.contains(&best) {
            println!("{}: Cycle: {:?}", count, best);
            break;
        } else {
            path.push(best)
        }
    }

    path
}