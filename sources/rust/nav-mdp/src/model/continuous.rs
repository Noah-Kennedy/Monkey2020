use crate::RobotVector;

#[derive(Default, Debug, Copy, Clone, PartialOrd, PartialEq)]
pub struct ContinuousState {
    pub position: RobotVector<f32>,
    pub velocity: RobotVector<f32>,
}

#[derive(Default, Debug)]
#[derive(PartialOrd, PartialEq)]
#[derive(Clone)]
pub struct ContinuousRobotParams {
    pub time_step: f32,
    pub length: f32,
    pub width: f32,
    pub max_turn_rate: f32,
    pub max_speed: f32,
    pub pos_res: f32,
    pub ang_res: f32,
}

#[derive(Default, Debug, Copy, Clone, PartialOrd, PartialEq)]
pub struct ContinuousAction {
    pub acceleration: RobotVector<f32>,
}