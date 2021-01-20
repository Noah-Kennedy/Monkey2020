#[macro_use]
extern crate derive_more;

pub use astar::*;
pub use forecast::*;
pub use model::*;
use nav_algo::mdp::value_iteration::ValueIterationParameters;
pub use perlin::*;

mod model;
mod forecast;
mod perlin;
mod astar;

pub const PERLIN: usize = 2;
pub const LENGTH: u16 = 160;
pub const WIDTH: u16 = 80;
pub const ANGLE_RES: u16 = 60;
pub const MAX_SPEED: u16 = 5;
pub const MIN_SPEED: u16 = 3;
pub const REV_MAX_SPEED: u16 = 0;
pub const REV_MIN_SPEED: u16 = 0;
pub const MAX_OMEGA: u16 = 5;
pub const GOAL: f32 = 0.0;

pub const START: DiscreteState = DiscreteState {
    position: RobotVector {
        x: (1 * WIDTH as i32 - 1) / 4,
        y: (1 * LENGTH as i32 - 1) / 4,
        r: (ANGLE_RES as i32 * 0) / 4
    }
};

pub const SPACE: RobotStateSpace = RobotStateSpace {
    length: LENGTH,
    width: WIDTH,
    ang_res: ANGLE_RES,
    max_omega: MAX_OMEGA,
    min_speed: MIN_SPEED,
    max_speed: MAX_SPEED,
    rev_min_speed: REV_MIN_SPEED,
    rev_max_speed: REV_MAX_SPEED,
    noise: 0.2,
    goal: RobotVector {
        x: (3 * WIDTH as i32 - 1) / 4,
        y: (3 * LENGTH as i32 - 1) / 4,
        r: (ANGLE_RES as i32 * 3) / 4,
    },
};

pub const PARAMS: ValueIterationParameters = ValueIterationParameters {
    epochs: 1000,
    discount: 0.90,
    living_reward: 0.0,
};
