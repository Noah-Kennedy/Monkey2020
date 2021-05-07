use std::os::raw;

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
    ResVGA60,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedDepthQuality {
    DepthPerformance,
    DepthQuality,
    DepthUltra,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMappingResolution {
    MapLowRes,
    MapMediumRes,
    MapHighRes,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMappingRange {
    MapNear,
    MapMedium,
    MapFar,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMeshFilter {
    FilterLow,
    FilterMedium,
    FilterHigh,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedSpatialMappingState {
    ZedMapInitializing,
    ZedMapOk,
    ZedMapNotEnoughMemory,
    ZedMapNotEnabled,
    ZedMapFpsTooLow,
}

/**************************************************************************************************
 * Functions
 *************************************************************************************************/

#[link(name = "monkey-vision")]
extern {
    /// Get positional data for an Aruco marker encoding a given ID value.
    ///
    /// # Params
    /// `aruco_id` - ID value encoded by a detected Aruco marker.
    ///
    /// # Returns
    /// `data` - Structure containing position data for the Aruco marker with the given ID.
    pub fn get_aruco_data(aruco_id: u16, data: *mut ArucoData, vision: *mut raw::c_void) -> bool;

    /// Get acceleration, orientation, and position data from the ZED IMU sensor.
    /// # Output
    /// Returns structure containing data from the ZED IMU.
    pub fn get_zed_imu_data(data: *mut ZedImuData, vision: *mut raw::c_void) -> bool;

    // TODO frame API when cody finishes it

    /// Set a flag to update the map mesh file the next time run_visual_processing() is called.
    /// NOTE: only update the mesh periodically as it is resource-intensive to do so.
    pub fn request_map_update(vision: *mut raw::c_void);

    /// Get the number of frames that have been processed.
    /// Returns integer count of the number of times run_visual_processing() has been called.
    pub fn get_frame_count(vision: *mut raw::c_void) -> u32;

    // TODO update
    /// Initialize camera
    /// # Arguments
    /// `mesh_path` - C-string path to save the stereo-scanned 3D environment mesh to.
    pub fn visual_processing_init(
        mesh_path: *const raw::c_char,
        camera_res: ZedCameraResolution,
        depth_quality: ZedDepthQuality,
        map_res: ZedMappingResolution,
        range: ZedMappingRange,
        mesh_filter: ZedMeshFilter,
    ) -> *mut raw::c_void;

    // todo update
    /// Run visual processing on a single video frame from the ZED camera.
    ///
    /// # Arguments
    /// marker_size - Width of the Aruco marker in meters.
    /// display - Boolean value indicating if the function should display the camera view in a GUI
    /// window.
    ///
    /// # Output
    /// Returns a boolean value indicating whether a camera frame could be successfully captured.
    /// `mapping_success` is set to whether or not map generation worked.
    pub fn run_visual_processing(
        marker_size: f32,
        display: bool,
        mapping_success: *mut bool,
        vision: *mut raw::c_void,
    ) -> bool;

    /// Deallocate all dynamic memory and close ZED camera.
    pub fn visual_processing_dealloc(vision: *mut raw::c_void);
}