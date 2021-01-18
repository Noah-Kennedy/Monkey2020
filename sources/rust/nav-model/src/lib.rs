#[macro_use]
extern crate derive_more;

pub use forecast::*;
pub use model::*;
pub use perlin::*;
use nav_algo::mdp::value_iteration::ValueIterationParameters;

mod model;
mod forecast;
mod perlin;

pub const LENGTH: u16 = 100;
pub const WIDTH: u16 = 100;
pub const ANGLE_RES: u16 = 8;
pub const MAX_SPEED: u16 = 5;
pub const MIN_SPEED: u16 = 2;
pub const MAX_OMEGA: u16 = 2;
pub const GOAL: f32 = 10.0;

pub const SPACE: RobotStateSpace = RobotStateSpace {
    length: LENGTH,
    width: WIDTH,
    ang_res: ANGLE_RES,
    max_omega: MAX_OMEGA,
    min_speed: MIN_SPEED,
    max_speed: MAX_SPEED,
    noise: 0.2,
    goal: RobotVector {
        x: (3 * WIDTH as i32 - 1) / 4,
        y: (3 * LENGTH as i32 - 1) / 4,
        r: 0,
    },
};

pub const PARAMS: ValueIterationParameters = ValueIterationParameters {
    epochs: 1000,
    discount: 0.90,
    living_reward: 0.0,
};
