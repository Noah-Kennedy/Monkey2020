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

/**************************************************************************************************
 * Aruco marker dictionary
 *************************************************************************************************/
const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
/**************************************************************************************************
 * ZED camera options
 *************************************************************************************************/
//VGA, HD720, HD1080, HD2K
const sl::RESOLUTION ZedCameraResolution = sl::RESOLUTION::VGA;
//VGA=100, HD720=60, HD1080=30, HD2K=15
const int ZedCameraFps = 100;
//PERFORMANCE, QUALITY, ULTRA 
const sl::DEPTH_MODE ZedDepthQuality = sl::DEPTH_MODE::ULTRA;
/**************************************************************************************************
 * ZED spatial mapping options
 *************************************************************************************************/
//MESH, FUSED_POINT_CLOUD
const sl::SpatialMappingParameters::SPATIAL_MAP_TYPE ZedMapType = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
//LOW=8cm, MEDIUM=5cm, HIGH=2cm
const float ZedMappingResolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
//NEAR=3.5m, MEDIUM=5m, FAR=10m
const float ZedMappingRange = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
//LOW, MEDIUM, HIGH
const sl::MeshFilterParameters::MESH_FILTER ZedMeshFilter = sl::MeshFilterParameters::MESH_FILTER::MEDIUM;


/**************************************************************************************************
 * C
 *************************************************************************************************/

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

extern "C" enum ZedCameraResolution {
    Res2K15,
    Res1080HD30,
    Res720HD60,
    ResVGA100
};

extern "C" enum ZedDepthQuality {
    DepthPERFORMANCE,
    DepthQUALITY,
    DepthULTRA
};

extern "C" enum ZedMappingResolution {
    MapLOWRES,
    MapMEDIUMRES,
    MapHIGHRES
};

extern "C" enum ZedMappingRange {
    MapNEAR,
    MapMEDIUM,
    MapFAR
};

extern "C" enum ZedMeshFilter {
    FilterLOW,
    FilterMEDIUM,
    FilterHIGH
};

/**************************************************************************************************
 * C++
 *************************************************************************************************/
namespace visual_processing {
    class MonkeyVision
    {
        public:
            MonkeyVision(std::string mesh_path, bool *success, sl::RESOLUTION camera_res, int fps, sl::DEPTH_MODE depth_quality, float map_res, float map_range, sl::MeshFilterParameters::MESH_FILTER filter);
            ~MonkeyVision();
            bool run(float marker_size, bool display);
            bool get_aruco(int id, ArucoData *data);
            bool get_imu(ZedImuData *data);
            bool get_frame(FrameBuffer *frame);
            long  frame_count();
            void update_map();
        private:
            // ZED camera handler
            sl::Camera zed;
            //ZED spatial mapping mesh
            std::string mesh_file_path;
            sl::Mesh map_mesh;
            sl::MeshFilterParameters::MESH_FILTER mesh_filter;
            bool update_map_mesh = false;
            // Vectors for storing detected Aruco marker IDs and positional data
            std::vector<int> detected_ids;
            std::vector<ArucoData> detected_markers;
            // Struct for storing ZED IMU data
            ZedImuData zed_imu_data;
            bool imu_valid = false;
            // Struct for storing the latest camera capture
            FrameBuffer camera_frame;
            // Frame counter for periodic tasks
            long frame_counter = 0;
            //
            float mapping_resolution;
            float mapping_range;
    };
}

/**************************************************************************************************
 * C
 *************************************************************************************************/

/**
 * @brief Get positional data for an Aruco marker encoding a given ID value.
 * @param aruco_id ID value encoded by a detected Aruco marker.
 * @return Structure containing position data for the Aruco marker with the given ID.
 */
extern "C" bool get_aruco_data(int aruco_id, ArucoData *data, visual_processing::MonkeyVision *vision);

/**
 * @brief Get acceleration, orientation, and position data from the ZED IMU sensor.
 * @return Structure containing data from the ZED IMU.
 */
extern "C" bool get_zed_imu_data(ZedImuData *data, visual_processing::MonkeyVision *vision);

/**
 * @brief Get the frame buffer from the last camera capture.
 * @return Structure containing a byte array of the image pixels and integers for the image dimensions, which will default to 0 if the camera image could not be captured. 
 */
extern "C" bool get_camera_frame(FrameBuffer *img, visual_processing::MonkeyVision *vision);

/**
 * @brief Set a flag to update the map mesh file the next time run_visual_processing() is called.
 *        NOTE: only update the mesh periodically as it is resource-intensive to do so.
 */
extern "C" void request_map_update(visual_processing::MonkeyVision *vision);

/**
 * @brief Get the number of frames that have been processed.
 * @return Integer count of the number of times run_visual_processing() has been called.
 */
extern "C" long get_frame_count(visual_processing::MonkeyVision *vision);

/**
 * @brief Intialize camera
 * @param mesh_path C-string path to save the stereo-scanned 3D environment mesh to.
 * @return Boolean value indicating whether the ZED camera was successfully initialized.
 */
extern "C" visual_processing::MonkeyVision* visual_processing_init(const char *mesh_path, bool *success, enum ZedCameraResolution camera_res, enum ZedDepthQuality depth_quality, enum ZedMappingResolution map_res, enum ZedMappingRange range, enum ZedMeshFilter mesh_filter);

/**
 * @brief Run visual processing on a single video frame from the ZED camera.
 * @param marker_size Width of the Aruco marker in meters.
 * @param display Boolean value indicating if the function should display the camera view in a GUI window.
 * @return Boolean value indicating whether a camera frame could be successfully captured. 
 */
extern "C" bool run_visual_processing(float marker_size, bool display, bool *mapping_success, visual_processing::MonkeyVision *vision);

/**
 * @brief Deallocate all dynamic memory and close ZED camera.
 */
extern "C" void visual_processing_dealloc(visual_processing::MonkeyVision *vision);
