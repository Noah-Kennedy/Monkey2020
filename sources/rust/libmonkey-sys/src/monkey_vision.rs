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
    DepthUltra
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMappingResolution {
    MapLowRes,
    MapMediumRes,
    MapHighRes
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMappingRange {
    MapNear,
    MapMedium,
    MapFar
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedMeshFilter {
    FilterLow,
    FilterMedium,
    FilterHigh
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum ZedSpatialMappingState {
    ZedMapInitializing,
    ZedMapOk,
    ZedMapNotEnoughMemory,
    ZedMapNotEnabled,
    ZedMapFpsTooLow
}

pub enum ZedErrorCode {
    ZedErrorSuccess, /**< Standard code for successful behavior.*/
    ZedErrorFailure, /**< Standard code for unsuccessful behavior.*/
    ZedErrorNoGpuCompatible, /**< No GPU found or CUDA capability of the device is not supported.*/
    ZedErrorNotEnoughGpuMemory, /**< Not enough GPU memory for this depth mode, try a different mode (such as PERFORMANCE), or increase the minimum depth value (see InitParameters::depth_minimum_distance).*/
    ZedErrorCameraNotDetected, /**< The ZED camera is not plugged or detected.*/
    ZedErrorSensorsNotAvailable, /**< a ZED-M or ZED2 camera is detected but the sensors (imu,barometer...) cannot be opened. Only for ZED-M or ZED2 devices*/
    ZedErrorInvalidResolution, /**< In case of invalid resolution parameter, such as a upsize beyond the original image size in Camera::retrieveImage */
    ZedErrorLowUsbBandwidth, /**< This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue).*/
    ZedErrorCalibrationFileNotAvailable, /**< ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.*/
    ZedErrorInvalidCalibrationFile, /**< ZED calibration file is not valid, try to download the factory one or recalibrate your camera using 'ZED Calibration'.*/
    ZedErrorInvalidSvoFile, /**< The provided SVO file is not valid.*/
    ZedErrorSvoRecordingError, /**< An recorder related error occurred (not enough free storage, invalid file).*/
    ZedErrorSvoUnsupportedCompression, /**< An SVO related error when NVIDIA based compression cannot be loaded.*/
    ZedErrorEndOfSVOFileReached, /**<SVO end of file has been reached, and no frame will be available until the SVO position is reset.*/
    ZedErrorInvalidCoordinateSystem, /**< The requested coordinate system is not available.*/
    ZedErrorInvalidFirmware, /**< The firmware of the ZED is out of date. Update to the latest version.*/
    ZedErrorInvalidFunctionParameters, /**< An invalid parameter has been set for the function. */
    ZedErrorCudaError, /**< In grab() or retrieveXXX() only, a CUDA error has been detected in the process. Activate verbose in sl::Camera::open for more info.*/
    ZedErrorCameraNotInitialized, /**< In grab() only, ZED SDK is not initialized. Probably a missing call to sl::Camera::open.*/
    ZedErrorNvidiaDriverOutOfDate, /**< Your NVIDIA driver is too old and not compatible with your current CUDA version. */
    ZedErrorInvalidFunctionCall, /**< The call of the function is not valid in the current context. Could be a missing call of sl::Camera::open. */
    ZedErrorCorruptedSdkInstallation, /**< The SDK wasn't able to load its dependencies or somes assets are missing, the installer should be launched. */
    ZedErrorIncompatibleSdkVersion, /**< The installed SDK is incompatible SDK used to compile the program. */
    ZedErrorInvalidAreaFile, /**< The given area file does not exist, check the path. */
    ZedErrorIncompatibleAreaFile, /**< The area file does not contain enought data to be used or the sl::DEPTH_MODE used during the creation of the area file is different from the one currently set. */
    ZedErrorCameraFailedToSetup, /**< Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.*/
    ZedErrorCameraDetectionIssue, /**< Your ZED can not be opened, try replugging it to another USB port or flipping the USB-C connector.*/
    ZedErrorCannotStartCameraStream, /**< Cannot start camera stream. Make sure your camera is not already used by another process or blocked by firewall or antivirus.*/
    ZedErrorNoGpuDetected, /**< No GPU found, CUDA is unable to list it. Can be a driver/reboot issue.*/
    ZedErrorPlaneNotFound, /**< Plane not found, either no plane is detected in the scene, at the location or corresponding to the floor, or the floor plane doesn't match the prior given*/
    ZedErrorModuleNotCompatibleWithCamera, /**< The Object detection module is only compatible with the ZED 2*/
    ZedErrorMotionSensorsRequired /**< The module needs the sensors to be enabled (see InitParameters::disable_sensors) */
}

/**************************************************************************************************
 * Functions
 *************************************************************************************************/

#[link(name = "monkey-vision")]
extern {
    // TODO get_aruco_data

    /// Get acceleration, orientation, and position data from the ZED IMU sensor.
    /// # Output
    /// Returns structure containing data from the ZED IMU.
    pub fn get_zed_imu_data(data: *mut ZedImuData, vision: *mut raw::c_void) -> bool;

    // TODO frame API when cody finishes it

    /// Set a flag to update the map mesh file the next time run_visual_processing() is called.
    /// NOTE: only update the mesh periodically as it is resource-intensive to do so.
    pub fn request_map_update(vision: *mut raw::c_void);

    // TODO get_frame_count

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
        vision: *mut raw::c_void
    ) -> bool;

    /// Deallocate all dynamic memory and close ZED camera.
    pub fn visual_processing_dealloc(vision: *mut raw::c_void);
}