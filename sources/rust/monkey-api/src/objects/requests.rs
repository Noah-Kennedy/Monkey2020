#[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LocationClass {
    CollectionBin,
    DiggingSite
}

#[derive(Default, Debug, PartialOrd, PartialEq, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RobotParams {
    pub max_turning_arc_radius: f32,
    pub drive_length: f32,
    pub drive_width: f32,
    pub wheel_radius: f32,
    pub max_speed: f32,
}
