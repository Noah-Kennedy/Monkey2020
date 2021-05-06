#[cfg(feature = "cameralot")]
pub mod cameralot;

/// Implement visual processing for Aruco fiducial marker detection using OpenCV and the ZED2
/// stereo camera system from Stereolabs as well as provide a C-style wrapper for Rust
/// interoperability.
#[cfg(feature = "monkey-vision")]
pub mod monkey_vision;