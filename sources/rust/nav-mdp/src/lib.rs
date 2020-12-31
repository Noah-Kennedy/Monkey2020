#[derive(Default, Debug, Copy, Clone, PartialOrd, PartialEq)]
pub struct SpatialVector {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

pub struct NavState {
    pub position: SpatialVector,
    pub velocity: SpatialVector,
}

pub struct NavStateTransitionTable {

}