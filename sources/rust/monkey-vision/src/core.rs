use std::ffi;

use crate::raw::*;

pub struct MonkeyVision {
    internal: *mut ffi::c_void,
}

pub enum MonkeyVisionError {
    Initialization,
    FetchZedData,
}

impl MonkeyVision {
    pub fn create(
        mesh_path: &str,
        camera_res: ZedCameraResolution,
        depth_quality: ZedDepthQuality,
        map_res: ZedMappingResolution,
        range: ZedMappingRange,
        mesh_filter: ZedMeshFilter,
    ) -> Result<MonkeyVision, MonkeyVisionError> {
        let path = ffi::CString::new(mesh_path).unwrap();

        let ret = unsafe {
            visual_processing_init(
                path.as_ptr(),
                camera_res,
                depth_quality,
                map_res,
                range,
                mesh_filter,
            )
        };

        if ret.is_null() {
            Err(MonkeyVisionError::Initialization)
        } else {
            Ok(Self {
                internal: ret
            })
        }
    }

    pub fn get_zed_imu_data(&mut self, data: &mut ZedImuData) -> Result<(), MonkeyVisionError> {
        let status = unsafe {
            get_zed_imu_data(data as *mut ZedImuData, self.internal)
        };

        if status {
            Ok(())
        } else {
            Err(MonkeyVisionError::FetchZedData)
        }
    }

    pub fn request_map_update(&mut self) {
        unsafe {
            request_map_update(self.internal);
        }
    }

    // TODO get_aruco_data
    // TODO run_visual_processing
    // TODO get_frame_count
    // TODO frame API when cody finishes it
}

impl Drop for MonkeyVision {
    fn drop(&mut self) {
        unsafe {
            visual_processing_dealloc(self.internal);
        }
    }
}