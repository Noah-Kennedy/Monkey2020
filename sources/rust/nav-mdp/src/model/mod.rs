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

pub use continuous::*;
pub use discrete::*;

mod discrete;
mod continuous;

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