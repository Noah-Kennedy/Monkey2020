/**
 * @project M.O.N.K.E.Y.
 * @date January 23 2021
 * @author Cody Nettesheim
 * @file visual_processing_c.h
 *
 * @brief Implement visual processing for AprilTag fiducial marker detection 
 * using OpenCV and a ZED2 stereo camera system.
 */
#pragma once
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/common/matd.h>

/**
 * @brief Structure to contain position data for an identified marker
 */
typedef struct {
    int x; /**< Image X-coordinate of the center of the AprilTag tag_data#x. */
    int y; /**< Image Y-coordinate of the center of the AprilTag tag_data#y. */
    float distance; /**< Estimated distance from the camera to the AprilTag tag_data#distance. */
    float pitch; /**< Pitch-axis rotation of the AprilTag tag_data#pitch. */
    float yaw; /**< Yaw-axis rotation of the AprilTag tag_data#yaw. */
    float roll; /**< Roll-axis rotation of the AprilTag tag_data#roll. */
} tag_data;

/**
 * @brief Intialize and allocate dynamic memory
 * @param dict Selection of pre-defined ArUco marker dictionary
 */
void visual_processing_init();

/**
 * @brief Detect AprilTags in an image 
 * @param img OpenCV image to scan
 * @return resizable array containing detected tags NOTE: caller is responsible for deallocating the zarray as visual_processing_dealloc() does not do so
 */
zarray_t* detect_tags(cv::Mat img);

/**
 * @brief Draw boundary around detected AprilTag
 * @param img Pointer to OpenCV image containing detected AprilTag
 * @param tag Pointer to detected AprilTag
 */
void draw_boundary(cv::Mat img, apriltag_detection_t *tag);

/**
 * @brief Process detected AprilTag and find positional data
 * @param tag Pointer to detected AprilTag
 * @param img ZED image capture where tag was detected
 * @return Structure containing position data 
 */
tag_data process_tag(apriltag_detection_t *tag, sl::Mat img);

/**
 * @brief Get point cloud distance to a single pixel of a ZED image capture
 * @param x Pixel X-coordinate
 * @param y Pixel Y-coordinate
 * @param img ZED SDK image matrix
 * @return Floating-point pixel depth value
 */
float get_pixel_depth(int x, int y, sl::Mat img);

/**
 * @brief Given the tag width, camera focal dimensions, and tag coordinates, generate a matrix for the estimated tag pose
 * @param tag Pointer to the AprilTag whose pose is being estimated
 * @param tag_size Tag width in meters
 * @param fx Camera focal width in pixels
 * @param fy Camera focal height in pixels
 * @param cx Tag X-coordinate
 * @param cy Tag Y-coordinate
 * @return Rotation matrix
 */
matd_t* get_tag_pose(apriltag_detection_t *tag, float tag_size, float fx, float fy, float cx, float cy);

/**
 * @brief Convert a ZED SDK matrix object to an OpenCV matrix
 * @param input ZED matrix
 * @return OpenCV matrix
 */
cv::Mat slMat2cvMat(sl::Mat& input);

/**
 * @brief Deallocate dynamic memory used in the visual processing
 */
void visual_processing_dealloc();

