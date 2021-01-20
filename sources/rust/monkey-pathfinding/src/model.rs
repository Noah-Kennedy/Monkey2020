//! The system dynamics is split between discrete and continuous modes.
//!
//! The discrete mode of operation is used internally by the navigation system for general
//! operation, and operates in "world units", which are tied to linear representations of atomic
//! units of resolution.
//!
//! Each unit represents one atomic unit of resolution within our system. These units are based
//! on our supplied resolutions for time, position, and orientation. These units can be
//! translated into a `Kg/m/s/rads` notation.
//!
//! This split has several advantages. First, avoiding floating point numbers has the advantage
//! of providing perfect precision and accuracy within our defined resolution. Second, avoiding
//! floating point numbers allows us to hash floating point numbers and compare equality in a
//! sound manner due to a lack of `NAN` values. Third, this provides faster calculations, as
//! FLOPs are more expensive than IOPs, and the IOPs performed consist solely of addition, and no
//! scaling or "snapping" to the nearest values within our resolution, or floating-point error
//! correction is required.
//!
//! Finally, this system makes it easier to reason about the actual size
//! of the state and action space, and allows for more error-free and readable code.

use std::f32::consts::TAU;

/// Vector container for robot state.
#[derive(Default, Debug, Hash)]
#[derive(Ord, PartialOrd, Eq, PartialEq)]
#[derive(Copy, Clone)]
#[derive(Add, Mul, AddAssign, MulAssign, Sub, SubAssign, DivAssign)]
pub struct RobotVector<T> {
    /// X component of vector, along the width of the field
    pub x: T,
    /// Y component of vector, along the length of the field.
    pub y: T,
    /// Angular component of vector
    pub r: T,
}

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
pub struct MonkeyModel {
    pub length: u16,
    pub width: u16,
    pub ang_res: u16,
    pub max_omega: u16,
    pub min_speed: u16,
    pub max_speed: u16,
    pub rev_min_speed: u16,
    pub rev_max_speed: u16,
}

impl MonkeyModel {
    fn validate_state(&self, state: &DiscreteState) -> bool {
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

    #[inline(never)]
    pub(crate) fn actions(&self, state: &DiscreteState) -> Vec<DiscreteAction> {
        let mut actions = Vec::new();

        for speed in (-(self.rev_max_speed as i32)..=-(self.rev_min_speed as i32))
            .chain((self.min_speed as i32)..=(self.max_speed as i32))
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

    pub fn states(&self) -> Vec<DiscreteState> {
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

                    states.push(s);
                }
            }
        }

        states
    }
}