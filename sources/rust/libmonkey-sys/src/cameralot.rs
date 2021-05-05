use std::os::raw;

#[repr(C)]
pub struct ByteBufferShare {
    pub buffer: *mut u8,
    pub length: usize,
}

#[repr(C)]
pub struct TimerData {
    pub grab_millis: u32,
    pub retrieve_millis: u32,
    pub resize_millis: u32,
    pub encode_millis: u32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
pub enum ReadStatus {
    Success = 0,
    NotOpen = 1,
    ReadFailed = 2,
    RetrieveFailed = 3,
    EmptyFrame = 4,
    EncodingFailed = 5,
}

#[link(name = "cameralot")]
extern {
    pub fn camera_feed_create() -> *mut raw::c_void;
    pub fn camera_feed_delete(feed: *mut raw::c_void);
    pub fn camera_feed_open(feed: *mut raw::c_void, index: i32) -> bool;
    pub fn camera_feed_open_api_pref(feed: *mut raw::c_void, index: i32, api: i32) -> bool;
    pub fn camera_feed_is_opened(feed: *mut raw::c_void) -> bool;
    pub fn camera_feed_read(
        feed: *mut raw::c_void,
        width: u32,
        height: u32,
        ext: *const raw::c_char,
        td: *mut TimerData,
    ) -> ReadStatus;
    pub fn camera_feed_get_buf(feed: *mut raw::c_void, buf: *mut ByteBufferShare) -> bool;
}