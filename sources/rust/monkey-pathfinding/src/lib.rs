#[macro_use]
extern crate derive_more;

pub mod model;
pub mod heap;
pub mod a_star;
#[cfg(feature = "demo")]
pub mod perlin;
pub mod state_space;
#[cfg(feature = "demo")]
pub mod demo_info;

pub type Discrete = i16;
const LIVING_COST: f32 = 0.000_000_1;