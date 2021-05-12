use std::ffi;
use std::sync::Arc;

use cameralot::core::{AbstractCameraFeed, CameraFeedError};
use cameralot::extend::abstract_camera_read_from_ffi;
use cameralot::prelude::TimerData;

use crate::raw::*;

const INIT_SUCCESS: InitErrorFlags = InitErrorFlags {
    camera_status_code: ZedStatusCode::ZedErrorSuccess,
    imu_status_code: ZedStatusCode::ZedErrorSuccess,
    map_status_code: ZedStatusCode::ZedErrorSuccess,
};

const RUNTIME_SUCCESS: RuntimeErrorFlags = RuntimeErrorFlags {
    camera_status_code: ZedStatusCode::ZedErrorSuccess,
    imu_status_code: ZedStatusCode::ZedErrorSuccess,
    map_status_code: ZedSpatialMappingState::ZedMapOk,
};

struct RawMonkey {
    internal: *mut ffi::c_void,
}

pub struct MonkeyVision {
    raw: Arc<RawMonkey>,
}

pub struct MonkeyCam {
    raw: Arc<RawMonkey>,
}

unsafe impl Send for MonkeyVision {}

unsafe impl Send for MonkeyCam {}

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum MonkeyVisionError {
    Init(InitErrorFlags),
    Runtime(RuntimeErrorFlags),
    ZedIMUData,
    ZedArucoData,
}

/// Initialize camera
/// # Arguments
/// `mesh_path` path to save the stereo-scanned 3D environment mesh to.
pub fn create(
    mesh_path: &str,
    camera_res: ZedCameraResolution,
    depth_quality: ZedDepthQuality,
    map_res: ZedMappingResolution,
    range: ZedMappingRange,
    mesh_filter: ZedMeshFilter,
)
    -> Result<(MonkeyVision, MonkeyCam), MonkeyVisionError>
{
    let path = ffi::CString::new(mesh_path).unwrap();
    let mut error = InitErrorFlags {
        camera_status_code: ZedStatusCode::ZedErrorSuccess,
        imu_status_code: ZedStatusCode::ZedErrorSuccess,
        map_status_code: ZedStatusCode::ZedErrorSuccess,
    };

    let ret = unsafe {
        visual_processing_init(
            path.as_ptr(),
            &mut error as *mut InitErrorFlags,
            camera_res,
            depth_quality,
            map_res,
            range,
            mesh_filter,
        )
    };

    if error == INIT_SUCCESS {
        let raw = Arc::new(RawMonkey { internal: ret });

        let vision = MonkeyVision {
            raw: raw.clone(),
        };

        let cam = MonkeyCam {
            raw: raw.clone(),
        };

        Ok((vision, cam))
    } else {
        Err(MonkeyVisionError::Init(error))
    }
}

impl MonkeyVision {
    pub fn aruco_data(&mut self, aruco_id: u16, data: &mut ArucoData) -> Result<(), MonkeyVisionError> {
        let status = unsafe {
            get_aruco_data(aruco_id, data as *mut ArucoData, self.raw.internal)
        };

        if status {
            Ok(())
        } else {
            Err(MonkeyVisionError::ZedArucoData)
        }
    }

    /// Get acceleration, orientation, and position data from the ZED IMU sensor.
    /// # Output
    /// Returns structure containing data from the ZED IMU.
    pub fn imu_data(&mut self, data: &mut ZedImuData) -> Result<(), MonkeyVisionError> {
        let status = unsafe {
            get_zed_imu_data(data as *mut ZedImuData, self.raw.internal)
        };

        if status {
            Ok(())
        } else {
            Err(MonkeyVisionError::ZedIMUData)
        }
    }

    /// Set a flag to update the map mesh file the next time run_visual_processing() is called.
    /// NOTE: only update the mesh periodically as it is resource-intensive to do so.
    pub fn request_map_update(&mut self) {
        unsafe {
            request_map_update(self.raw.internal);
        }
    }

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
    pub fn run(&mut self, marker_size: f32, display: bool) -> Result<(), MonkeyVisionError> {
        let mut status = RuntimeErrorFlags {
            camera_status_code: ZedStatusCode::ZedErrorSuccess,
            imu_status_code: ZedStatusCode::ZedErrorSuccess,
            map_status_code: ZedSpatialMappingState::ZedMapInitializing,
        };

        unsafe {
            run_visual_processing(
                marker_size,
                display,
                (&mut status) as *mut RuntimeErrorFlags,
                self.raw.internal,
            )
        }

        if status == RUNTIME_SUCCESS {
            Ok(())
        } else {
            Err(MonkeyVisionError::Runtime(status))
        }
    }

    // TODO get_frame_count
    // TODO frame API when cody finishes it
}

impl AbstractCameraFeed for MonkeyCam {
    unsafe fn read(&mut self, width: u32, height: u32, ext: &str, td: &mut TimerData) -> Result<&[u8], CameraFeedError> {
        abstract_camera_read_from_ffi(self.raw.internal, width, height, ext, td)
    }
}

impl Drop for RawMonkey {
    fn drop(&mut self) {
        unsafe {
            visual_processing_dealloc(self.internal);
        }
    }
}