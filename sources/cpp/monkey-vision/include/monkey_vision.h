/**
 * @project M.O.N.K.E.Y.
 * @date January 23 2021
 * @author Cody Nettesheim
 * @file visual_processing_c.h
 *
 * @brief Implement visual processing for Aruco fiducial marker detection 
 * using OpenCV and the ZED2 stereo camera system from Stereolabs as well as
 * provide a C-style wrapper for Rust interoperability.
 */
#pragma once

/**
 * @brief C-style enum to wrap the RESOLUTION enum class used by the ZED library.
 */
extern "C" enum ZedResolution
{
    Res2K, /**< Maps to RESOLUTION::HD2K in the ZED library and runs at 2K, 15fps*/
    Res1080P, /**< Maps to RESOLUTION::HD1080 and runs at 1080p, 30fps */
    Res720P, /**< Maps to RESOLUTION::HD720 and runs at 720p, 60fps */
    ResUSB2 /**< Maps to RESOLUTION::VGA and runs at 672x376, 30fps; this is the only usable video mode for a ZED connected over a USB2.0 interface */
};

/**
 * @brief C-style enum to wrap the DEPTH_MODE enum class used by the ZED library.
 */
extern "C" enum ZedDepthQuality
{
    PERFORMANCE_DEPTH, /**< Maps to DEPTH_MODE::PERFORMANCE */
    QUALITY_DEPTH, /**< Maps to DEPTH_MODE:: */
    ULTRA_DEPTH /**< Maps to DEPTH_MODE::ULTRA */
};

/**
 * @brief Structure to contain position data for an identified Aruco marker. All members default to -1 in case no Aruco marker was detected.
 */
extern "C" struct ArucoData {
    float distance = -1; /**< Stereo-measured scalar distance to the marker in meters */
    float x_dist = -1; /**< Pose-estimated X-axis distance to marker in meters */
    float y_dist = -1; /**< Pose-estimated Y-axis distance to marker in meters */
    float z_dist = -1; /**< Pose-estimated Z-axis distance to marker in meters */
    float x_rot = -1; /**< Pose-estimated X-axis rotation of the marker in degrees */
    float y_rot = -1; /**< Pose-estimated Y-axis rotation of marker in degrees */
    float z_rot = -1; /**< Pose-estimated Z-axis rotation of marker in degrees */
};

/**
 * @brief Structure to contain acceleration and orientation data measured from the ZED IMU. All members default to -1 in case the IMU could not be accessed.
 */
extern "C" struct ZedImuData {
    float x_accel = -1; /**< Linear acceleration of the ZED along the X-axis in m/s^2 */
    float y_accel = -1; /**< Linear acceleration of the ZED along the Y-axis in m/s^2 */
    float z_accel = -1; /**< Linear acceleration of the ZED along the Z-axis in m/s^2 */
    float x_rot_accel = -1; /**< Angular acceleration of the ZED around the X-axis in deg/s */
    float y_rot_accel = -1; /**< Angular accleration of the ZED around the Z-axis in deg/s */
    float z_rot_accel = -1; /**< Angular acceleration of the ZED around the Z-axis in deg/s */
};

/**
 * @brief Structure to store a camera frame as a byte array with dimensions
 */
extern "C" struct FrameBuffer {
    uchar *buffer; /**< Byte array which stores the image frame */
    uint32_t width = 0; /**< Width of the image in pixels */
    uint32_t height = 0; /**< Height of the image in pixels */
}

/**
 * @brief Intialize camera
 * @param camera_res Enum to select the resolution at which the ZED camera should run.
 * @param depth Enum to select the depth mode which the ZED should use.
 *        NOTE: Higher quality depth measurement will use more GPU resources.
 * @return Boolean value indicating whether the ZED camera was successfully initialized.
 */
extern "C" bool visual_processing_init(ZedResolution camera_res, ZedDepthQuality depth);

/**
 * @brief Get positional data for an Aruco marker encoding a given ID value.
 * @param aruco_id ID value encoded by a detected Aruco marker.
 * @return Structure containing position data for the Aruco marker with the given ID. Struct will have values of -1 if an Aruco marker with the given ID was not detected.
 */
extern "C" ArucoData get_aruco_data(int aruco_id);

/**
 * @brief Get acceleration, orientation, and position data from the ZED IMU sensor.
 * @return Structure containing data from the ZED IMU. Struct will have values of -1 if the ZED IMU could not be accessed.
 */
extern "C" ZedImuData get_zed_imu_data();

/**
 * @brief Get the frame buffer from the last camera capture.
 * @return Structure containing a byte array of the image pixels and integers for the image dimensions, which will default to 0 if the camera image could not be captured. 
 */
extern "C" FrameBuffer get_camera_frame();

/**
 * @brief Run visual processing on a single video frame from the ZED camera.
 * @param marker_size Width of the Aruco marker in meters.
 * @param mesh_path C-string path to save the stereo-scanned 3D environment mesh to.
 * @param display Boolean value indicating if the function should display the camera view in a GUI window.
 * @return ArucoData struct for the aruco with the given ID. If no marker was detected with the given ID, return a struct with values of -1 for all parameters. 
 */
extern "C" bool run_visual_processing(float marker_size, const char *mesh_path, bool display);

/**
 * @brief Deallocate all dynamic memory and close ZED camera.
 */
extern "C" void visual_processing_dealloc();

