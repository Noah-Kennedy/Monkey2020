use std::ffi::CString;
use std::os::raw;
use std::ptr::null_mut;
use std::slice::from_raw_parts;

use crate::core::CameraFeedError;
use crate::raw::*;

pub unsafe fn abstract_camera_read_from_ffi<'a>(
    internal: *mut raw::c_void, width: u32, height: u32, ext: &str, td: &mut TimerData,
) -> Result<&'a [u8], CameraFeedError> {
    let c_str = CString::new(ext).unwrap();

    let mut share = ByteBufferShare { buffer: null_mut(), length: 0 };

    let status = abstract_camera_feed_read(
        internal,
        width, height,
        c_str.as_ptr(),
        td as *mut TimerData,
        (&mut share) as *mut ByteBufferShare,
    );

    match status {
        ReadStatus::Success => Ok(from_raw_parts(share.buffer, share.length)),
        ReadStatus::NotOpen => Err(CameraFeedError::NotOpen),
        ReadStatus::ReadFailed => Err(CameraFeedError::ReadFailed),
        ReadStatus::RetrieveFailed => Err(CameraFeedError::RetrieveFailed),
        ReadStatus::EmptyFrame => Err(CameraFeedError::EmptyFrame),
        ReadStatus::EncodingFailed => Err(CameraFeedError::EncodingFailed),
    }
}