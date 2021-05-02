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
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

/**
 * @brief C-style enum to wrap the RESOLUTION enum class used by the ZED library
 */
typedef enum {
    2K, /**< Maps to RESOLUTION::HD2K in the ZED library and runs at 2K, 15fps*/
    1080P, /**< Maps to RESOLUTION::HD1080 and runs at 1080p, 30fps */
    720P, /**< Maps to RESOLUTION::HD720 and runs at 720p, 60fps */
    USB2 /**< Maps to RESOLUTION::VGA and runs at 672x376, 30fps; this is the only usable video mode for a ZED connected over a USB2.0 interface */
} zed_resolution;

/**
 * @brief C-style enum to wrap the DEPTH_MODE enum class used by the ZED library
 */
typedef enum {
    PERFORMANCE_DEPTH, /**< Maps to DEPTH_MODE::PERFORMANCE */
    QUALITY_DEPTH, /**< Maps to DEPTH_MODE:: */
    ULTRA_DEPTH /**< Maps to DEPTH_MODE::ULTRA */
} zed_depth_quality;

/**
 * @brief Structure to contain position data for an identified Aruco marker.
 *        Acts as a linked list node.
 */
typedef struct {
    int id; //ID value encoded by the marker
    float distance; /**< Stereo-measured scalar distance to the marker */
    float x_dist; /**< Pose-estimated X-axis distance to marker */
    float y_dist; /**< Pose-estimated Y-axis distance to marker */
    float z_dist; /**< Pose-estimated Z-axis distance to marker */
    float x_rot; /**< Pose-estimated X-axis rotation of the marker in degrees */
    float y_rot; /**< Pose-estimated Y-axis rotation of marker */
    float z_rot; /**< Pose-estimated Z-axis rotation of marker */
    aruco_data* next = NULL; /**< Pointer to next node in the list */
} aruco_data;

/**
 * @brief Intialize camera
 * @param camera_res Enum to select the resolution at which the ZED camera should run
 * @param depth Enum to select the depth mode which the ZED should use 
 *        NOTE: Higher quality depth measurement will use more GPU resources 
 * @return Boolean value indicating whether the ZED camera was successfully initialized
 */
bool visual_processing_init(zed_resolution camera_res, zed_depth_quality depth);

/**
 * @brief Run visual processing on a single video frame from the ZED camera; this function should be called in a loop
 * @param aruco_list Pointer to Aruco linked list head 
 * @param display Boolean value indicating if the function should display the camera view in a GUI window
 * @param marker_size Width of the Aruco markers in meters
 * @return Boolean value indicating whether a frame could be successfully captured from the ZED camera
 */
bool run_visual_processing(aruco_data* aruco_list, bool display, float marker_size);

/**
 * @brief Deallocate Aruco data list
 * @param head Pointer to list head 
 */
void aruco_delete(aruco_data* head);

