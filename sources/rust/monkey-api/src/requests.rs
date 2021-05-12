use std::time::Duration;
use crate::Location;

#[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LocationClass {
    CollectionBin,
    DiggingSite,
}

#[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CameraRequest {
    pub id: u64,
}

#[derive(Debug, Default, PartialOrd, PartialEq, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AutonomousParams {
    pub target: Option<Location>,
    pub max_speed: f32,
    pub max_force: f32,
    pub mass: f32,
    pub moment_of_inertia: f32,
    pub allow_backwards: bool,
    pub stopping_dist: f32,
    pub interaction_radius: f32,
    pub min_x: f32,
    pub max_x: f32,
    pub min_z: f32,
    pub max_z: f32,
    pub res_x: u16,
    pub res_z: u16,
    pub min_mesh_to_grid_period: Duration,
    pub vertical_cutoff: f32,
    pub min_turn_radius: f32,
    pub drive_width: f32,
    pub wheel_radius: f32,
}
