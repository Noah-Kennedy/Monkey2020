#[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LocationClass {
    CollectionBin,
    DiggingSite
}

#[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CameraRequest {
    pub id: u64,
}

#[derive(Default, Debug, PartialOrd, PartialEq, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AutonomousParams {
    /// In milliseconds
    pub min_spatial_map_update_period: usize,
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
    pub vertical_cutoff: f32,
    pub min_turn_radius: f32,
    pub drive_width: f32,
    pub wheel_radius: f32
}

// Parameters defining autonomous behavior
// const MIN_SPATIAL_MAP_UPDATE_PERIOD: usize = 500;
// const MAX_SPEED: f32 = 100.0;
// const MAX_FORCE: f32 = 100.0;
// const MASS: f32 = 1.0;
// const MOMENT_OF_INERTIA: f32 = 1.0;
// const ALLOW_BACKWARDS: bool = false;
// const STOPPING_DIST: f32 = 100.0;
// const INTERACTION_RADIUS: f32 = 0.2;
// const MIN_X: f32 = -5.0;
// const MAX_X: f32 = 1.0;
// const MIN_Z: f32 = -7.0;
// const MAX_Z: f32 = 0.0;
// const RES_X: usize = 128;
// const RES_Z: usize = 128;
// const VERTICAL_CUTOFF: f32 = 1.0;
