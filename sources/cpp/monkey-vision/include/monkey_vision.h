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
#include <stdbool.h>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

//#define _DEBUG_

/*** Aruco marker dictionary ***/
const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
/*** ZED camera options ***/
//VGA, HD720, HD1080, HD2K
const sl::RESOLUTION ZedCameraResolution = sl::RESOLUTION::VGA;
//VGA=100, HD720=60, HD1080=30, HD2K=15
const int ZedCameraFps = 100;
//PERFORMANCE, QUALITY, ULTRA 
const sl::DEPTH_MODE ZedDepthQuality = sl::DEPTH_MODE::ULTRA;

/*** ZED spatial mapping options ***/
//MESH, FUSED_POINT_CLOUD
const sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZedMapType = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
//LOW=8cm, MEDIUM=5cm, HIGH=2cm
const float ZedMappingResolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
//NEAR=3.5m, MEDIUM=5m, FAR=10m
const float ZedMappingRange = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
//LOW, MEDIUM, HIGH
const sl::MeshFilterParameters::MESH_FILTER ZedMeshFilter = sl::MeshFilterParameters::MESH_FILTER::MEDIUM;

/**
 * @brief Structure to contain position data for an identified Aruco marker.
 */
extern "C" struct ArucoData {
    float x_dist = 0; /**< Pose-estimated X-axis distance to marker in meters */
    float y_dist = 0; /**< Pose-estimated Y-axis distance to marker in meters */
    float z_dist = 0; /**< Pose-estimated Z-axis distance to marker in meters */
    float x_rot = 0; /**< Pose-estimated X-axis rotation of the marker in degrees */
    float y_rot = 0; /**< Pose-estimated Y-axis rotation of marker in degrees */
    float z_rot = 0; /**< Pose-estimated Z-axis rotation of marker in degrees */
};

/**
 * @brief Structure to contain acceleration and orientation data measured from the ZED IMU.
 */
extern "C" struct ZedImuData {
    float x_accel = 0; /**< Linear acceleration of the ZED along the X-axis in m/s^2 */
    float y_accel = 0; /**< Linear acceleration of the ZED along the Y-axis in m/s^2 */
    float z_accel = 0; /**< Linear acceleration of the ZED along the Z-axis in m/s^2 */
    float x_rot_vel = 0; /**< Angular velocity of the ZED around the X-axis in deg/s */
    float y_rot_vel = 0; /**< Angular velocity of the ZED around the Z-axis in deg/s */
    float z_rot_vel = 0; /**< Angular velocity of the ZED around the Z-axis in deg/s */
    float x_pos = 0; /**< Estimated X-axis displacement of the ZED relative to the origin in meters */
    float y_pos = 0; /**< Estimated Y-axis displacement of the ZED relative to the origin in meters */
    float z_pos = 0; /**< Estimated Z-axis displacement of the ZED relative to the origin in meters */
    float x_rot = 0; /**< X-axis orientation of the ZED in degrees */
    float y_rot = 0; /**< Y-axis orientation of the ZED in degrees */
    float z_rot = 0; /**< Z-axis orientation of the ZED in degrees */
};

/**
 * @brief Structure to store a camera frame as a byte array with dimensions
 */
extern "C" struct FrameBuffer {
    uchar *buffer; /**< Byte array which stores the image frame */
    uint32_t width = 0; /**< Width of the image in pixels */
    uint32_t height = 0; /**< Height of the image in pixels */
};

/**
 * @brief Get positional data for an Aruco marker encoding a given ID value.
 * @param aruco_id ID value encoded by a detected Aruco marker.
 * @return Structure containing position data for the Aruco marker with the given ID.
 */
extern "C" ArucoData get_aruco_data(int aruco_id);

/**
 * @brief Get acceleration, orientation, and position data from the ZED IMU sensor.
 * @return Structure containing data from the ZED IMU.
 */
extern "C" ZedImuData get_zed_imu_data();

/**
 * @brief Get the frame buffer from the last camera capture.
 * @return Structure containing a byte array of the image pixels and integers for the image dimensions, which will default to 0 if the camera image could not be captured. 
 */
extern "C" FrameBuffer get_camera_frame();

/**
 * @brief Set a flag to update the map mesh file the next time run_visual_processing() is called.
 *        NOTE: only update the mesh periodically as it is resource-intensive to do so.
 */
extern "C" void request_map_update();

/**
 * @brief Get the number of frames that have been processed.
 * @return Integer count of the number of times run_visual_processing() has been called.
 */
extern "C" long get_frame_count();

/**
 * @brief Intialize camera
 * @param mesh_path C-string path to save the stereo-scanned 3D environment mesh to.
 * @return Boolean value indicating whether the ZED camera was successfully initialized.
 */
extern "C" bool visual_processing_init(const char *mesh_path);

/**
 * @brief Run visual processing on a single video frame from the ZED camera.
 * @param marker_size Width of the Aruco marker in meters.
 * @param display Boolean value indicating if the function should display the camera view in a GUI window.
 * @return Boolean value indicating whether a camera frame could be successfully captured. 
 */
extern "C" bool run_visual_processing(float marker_size, bool display);

/**
 * @brief Deallocate all dynamic memory and close ZED camera.
 */
extern "C" void visual_processing_dealloc();

