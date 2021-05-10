pub mod responses;

pub mod requests;

#[derive(Default, Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Location {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[derive(Default, Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MotorSpeeds {
    /// The speed for the right motor, in radians per second
    pub right_speed: f32,
    /// The speed for the left motor, in radians per second
    pub left_speed: f32,
}
