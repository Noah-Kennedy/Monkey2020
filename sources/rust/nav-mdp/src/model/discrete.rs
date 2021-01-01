use crate::RobotVector;

#[derive(Default, Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub struct DiscreteState {
    pub position: RobotVector<i32>,
    pub velocity: RobotVector<i32>,
}

#[derive(Default, Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub struct DiscreteAction {
    pub acceleration: RobotVector<i32>,
}

#[derive(Default, Debug)]
#[derive(PartialOrd, PartialEq)]
#[derive(Clone)]
pub struct DiscreteRobotParams {
    pub length: u16,
    pub width: u16,
    pub turn_rate: u16,
    pub num_angles: u16,
    pub num_angular_speeds: u16,
    pub num_speeds: u16,
}

impl DiscreteState {
    pub fn apply(&self, action: DiscreteAction) -> Self {
        Self {
            position: self.position + self.velocity,
            velocity: self.velocity + action.acceleration,
        }
    }
}