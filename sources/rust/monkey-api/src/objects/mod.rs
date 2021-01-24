pub mod responses;

pub mod requests;

#[derive(Default, Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Location {
    pub x: f32,
    pub y: f32,
    pub theta: Option<f32>,
}