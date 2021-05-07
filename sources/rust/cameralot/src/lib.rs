pub use libmonkey_sys::cameralot as raw;

pub mod prelude {
    pub use crate::core::*;
    pub use crate::raw::TimerData;
}

pub mod extend;

pub mod core {
    use std::os::raw;

    use crate::extend::*;
    use crate::raw::*;

    pub trait AbstractCameraFeed {
        /// # Safety
        /// This function is only safe if there are no living references to the camera feed buffer.
        unsafe fn read(
            &mut self, width: u32, height: u32, ext: &str, td: &mut TimerData,
        ) -> Result<&[u8], CameraFeedError>;
    }

    pub trait CameraFeed: AbstractCameraFeed {
        fn is_opened(&self) -> bool;
    }

    pub struct OpenCVCameraFeed {
        internal: *mut raw::c_void,
    }

    #[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
    pub enum CameraFeedError {
        NotOpen,
        ReadFailed,
        RetrieveFailed,
        EmptyFrame,
        EncodingFailed,
        NoFrame,
    }

    impl OpenCVCameraFeed {
        pub fn new() -> Self {
            Self {
                internal: unsafe { opencv_camera_feed_create() }
            }
        }

        pub fn open(&mut self, index: i32) -> bool {
            unsafe {
                opencv_camera_feed_open(self.internal, index)
            }
        }

        pub fn open_api_pref(&mut self, index: i32, api: i32) -> bool {
            unsafe {
                opencv_camera_feed_open_api_pref(self.internal, index, api)
            }
        }
    }

    impl CameraFeed for OpenCVCameraFeed {
        fn is_opened(&self) -> bool {
            unsafe {
                camera_feed_is_opened(self.internal)
            }
        }
    }

    impl AbstractCameraFeed for OpenCVCameraFeed {
        unsafe fn read(
            &mut self, width: u32, height: u32, ext: &str, td: &mut TimerData,
        )
            -> Result<&[u8], CameraFeedError>
        {
            abstract_camera_read_from_ffi(self.internal, width, height, ext, td)
        }
    }

    impl Drop for OpenCVCameraFeed {
        fn drop(&mut self) {
            unsafe {
                opencv_camera_feed_delete(self.internal);
            }
        }
    }
}