/**************************************************************************************************
 * Structs
 *************************************************************************************************/

/// Structure to contain position data for an identified Aruco marker.
#[repr(C)]
#[derive(Clone, PartialOrd, PartialEq, Debug, Default)]
pub struct ArucoData {
    /// Pose-estimated X-axis distance to marker in meters
    pub x_dist: f32,
    /// Pose-estimated Y-axis distance to marker in meters
    pub y_dist: f32,
    /// Pose-estimated Z-axis distance to marker in meters
    pub z_dist: f32,
    /// Pose-estimated X-axis rotation of the marker in radians
    pub x_rot: f32,
    /// Pose-estimated Y-axis rotation of marker in radians
    pub y_rot: f32,
    /// Pose-estimated Z-axis rotation of marker in radians
    pub z_rot: f32,
}

/// Structure to contain acceleration and orientation data measured from the ZED IMU.
#[repr(C)]
#[derive(Clone, PartialOrd, PartialEq, Debug, Default)]
pub struct ZedImuData {
    /// Linear acceleration of the ZED along the X-axis in m/s^2
    pub x_accel: f32,
    /// Linear acceleration of the ZED along the Y-axis in m/s^2
    pub y_accel: f32,
    /// Linear acceleration of the ZED along the Z-axis in m/s^2
    pub z_accel: f32,
    /// Angular velocity of the ZED around the X-axis in deg/s
    pub x_rot_vel: f32,
    /// Angular velocity of the ZED around the Z-axis in deg/s
    pub y_rot_vel: f32,
    /// Angular velocity of the ZED around the Z-axis in deg/s
    pub z_rot_vel: f32,
    /// Estimated X-axis displacement of the ZED relative to the origin in meters
    pub x_pos: f32,
    /// Estimated Y-axis displacement of the ZED relative to the origin in meters
    pub y_pos: f32,
    /// Estimated Z-axis displacement of the ZED relative to the origin in meters
    pub z_pos: f32,
    /// X-axis orientation of the ZED in degrees
    pub x_rot: f32,
    /// Y-axis orientation of the ZED in degrees
    pub y_rot: f32,
    /// Z-axis orientation of the ZED in degrees
    pub z_rot: f32,
}

/// Structure to store a camera frame as a byte array with dimensions
#[repr(C)]
#[derive(Debug)]
pub struct FrameBuffer {
    pub buffer: *mut u8,
    pub width: u32,
    pub height: u32,
}

/**************************************************************************************************
 * Enums
 *************************************************************************************************/

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedCameraResolution {
    Res2K15,
    Res1080HD30,
    Res720HD60,
    ResVGA100,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedDepthQuality {
    DepthPERFORMANCE,
    DepthQUALITY,
    DepthULTRA,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMappingResolution {
    MapLOWRES,
    MapMEDIUMRES,
    MapHIGHRES,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMappingRange {
    MapNEAR,
    MapMEDIUM,
    MapFAR,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMeshFilter {
    FilterLOW,
    FilterMEDIUM,
    FilterHIGH,
}

/**************************************************************************************************
 * Functions
 *************************************************************************************************/

#[link(name = "monkey-vision")]
extern {
    // TODO
}