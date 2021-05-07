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
#include <camera_feed.h>

/**************************************************************************************************
 * C
 *************************************************************************************************/

/// Structure to contain position data for an identified Aruco marker.
extern "C" struct ArucoData
{
    /// Pose-estimated X-axis distance to marker in meters
    float x_dist = 0;
    /// Pose-estimated Y-axis distance to marker in meters
    float y_dist = 0;
    /// Pose-estimated Z-axis distance to marker in meters
    float z_dist = 0;
    /// Pose-estimated X-axis rotation of the marker in radians
    float x_rot = 0;
    /// Pose-estimated Y-axis rotation of marker in radians
    float y_rot = 0;
    /// Pose-estimated Z-axis rotation of marker in radians
    float z_rot = 0;
};

 /// Structure to contain acceleration and orientation data measured from the ZED IMU.
extern "C" struct ZedImuData
{
    /// Linear acceleration of the ZED along the X-axis in m/s^2
    float x_accel = 0;
    /// Linear acceleration of the ZED along the Y-axis in m/s^2
    float y_accel = 0;
    /// Linear acceleration of the ZED along the Z-axis in m/s^2
    float z_accel = 0;
    /// Angular velocity of the ZED around the X-axis in deg/s
    float x_rot_vel = 0;
    /// Angular velocity of the ZED around the Z-axis in deg/s
    float y_rot_vel = 0;
    /// Angular velocity of the ZED around the Z-axis in deg/s
    float z_rot_vel = 0;
    /// Estimated X-axis displacement of the ZED relative to the origin in meters
    float x_pos = 0;
    /// Estimated Y-axis displacement of the ZED relative to the origin in meters
    float y_pos = 0;
    /// Estimated Z-axis displacement of the ZED relative to the origin in meters
    float z_pos = 0;
    /// X-axis orientation of the ZED in degrees
    float x_rot = 0;
    /// Y-axis orientation of the ZED in degrees
    float y_rot = 0;
    /// Z-axis orientation of the ZED in degrees
    float z_rot = 0;
};

extern "C" enum ZedCameraResolution
{
    Res2K15,
    Res1080HD30,
    Res720HD60,
    ResVGA60
};

extern "C" enum ZedDepthQuality
{
    DepthPerformance,
    DepthQuality,
    DepthUltra
};

extern "C" enum ZedMappingResolution
{
    MapLowRes,
    MapMediumRes,
    MapHighRes
};

extern "C" enum ZedMappingRange
{
    MapNear,
    MapMedium,
    MapFar
};

extern "C" enum ZedMeshFilter
{
    FilterLow,
    FilterMedium,
    FilterHigh
};

extern "C" enum ZedSpatialMappingState
{
    ZedMapInitializing,
    ZedMapOk,
    ZedMapNotEnoughMemory,
    ZedMapNotEnabled,
    ZedMapFpsTooLow
};

extern "C" enum ZedStatusCode
{
    /// Standard code for successful behavior.
    ZedErrorSuccess,
    /// Standard code for unsuccessful behavior.
    ZedErrorFailure,
    /// No GPU found or CUDA capability of the device is not supported.
    ZedErrorNoGpuCompatible,
    /// Not enough GPU memory for this depth mode, try a different mode (such as PERFORMANCE), or increase the minimum depth value (see InitParameters::depth_minimum_distance).
    ZedErrorNotEnoughGpuMemory,
    /// The ZED camera is not plugged or detected.
    ZedErrorCameraNotDetected,
    /// a ZED-M or ZED2 camera is detected but the sensors (imu,barometer...) cannot be opened. Only for ZED-M or ZED2 devices
    ZedErrorSensorsNotAvailable,
    /// In case of invalid resolution parameter, such as a upsize beyond the original image size in Camera::retrieveImage
    ZedErrorInvalidResolution,
    /// This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue).
    ZedErrorLowUsbBandwidth,
    /// ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.
    ZedErrorCalibrationFileNotAvailable,
    /// ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.
    ZedErrorInvalidCalibrationFile,
    /// The provided SVO file is not valid.
    ZedErrorInvalidSvoFile,
    /// An recorder related error occurred (not enough free storage, invalid file).
    ZedErrorSvoRecordingError,
    /// An SVO related error when NVIDIA based compression cannot be loaded.
    ZedErrorSvoUnsupportedCompression,
    /// SVO end of file has been reached, and no frame will be available until the SVO position is reset.
    ZedErrorEndOfSVOFileReached,
    /// The requested coordinate system is not available.
    ZedErrorInvalidCoordinateSystem,
    /// The firmware of the ZED is out of date. Update to the latest version.
    ZedErrorInvalidFirmware,
    /// An invalid parameter has been set for the function.
    ZedErrorInvalidFunctionParameters,
    /// In grab() or retrieveXXX() only, a CUDA error has been detected in the process. Activate verbose in sl::Camera::open for more info.
    ZedErrorCudaError,
    /// In grab() only, ZED SDK is not initialized. Probably a missing call to sl::Camera::open.
    ZedErrorCameraNotInitialized,
    /// Your NVIDIA driver is too old and not compatible with your current CUDA version.
    ZedErrorNvidiaDriverOutOfDate,
    /// The call of the function is not valid in the current context. Could be a missing call of sl::Camera::open.
    ZedErrorInvalidFunctionCall,
    /// The SDK wasn't able to load its dependencies or somes assets are missing, the installer should be launched.
    ZedErrorCorruptedSdkInstallation,
    /// The installed SDK is incompatible SDK used to compile the program.
    ZedErrorIncompatibleSdkVersion,
    /// The given area file does not exist, check the path.
    ZedErrorInvalidAreaFile,
    /// The area file does not contain enought data to be used or the sl::DEPTH_MODE used during the creation of the area file is different from the one currently set.
    ZedErrorIncompatibleAreaFile,
    /// Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.
    ZedErrorCameraFailedToSetup,
    /// Your ZED can not be opened, try replugging it to another USB port or flipping the USB-C connector.
    ZedErrorCameraDetectionIssue,
    /// Cannot start camera stream. Make sure your camera is not already used by another process or blocked by firewall or antivirus.
    ZedErrorCannotStartCameraStream,
    /// No GPU found, CUDA is unable to list it. Can be a driver/reboot issue.
    ZedErrorNoGpuDetected,
    /// Plane not found, either no plane is detected in the scene, at the location or corresponding to the floor, or the floor plane doesn't match the prior given
    ZedErrorPlaneNotFound,
    /// The Object detection module is only compatible with the ZED 2
    ZedErrorModuleNotCompatibleWithCamera,
    /// The module needs the sensors to be enabled (see InitParameters::disable_sensors)
    ZedErrorMotionSensorsRequired
};

extern "C" struct InitErrorFlags
{
    ZedStatusCode camera_status_code;
    ZedStatusCode imu_status_code;
    ZedStatusCode map_status_code;
};

extern "C" struct RuntimeErrorFlags
{
    ZedStatusCode camera_status_code;
    ZedStatusCode imu_status_code;
    ZedSpatialMappingState map_status_code;
};

/**************************************************************************************************
 * C++
 *************************************************************************************************/
namespace visual_processing {
    class MonkeyVision
    {
    public:
        MonkeyVision(std::string mesh_path, InitErrorFlags *error_codes, sl::RESOLUTION camera_res, uint8_t fps, sl::DEPTH_MODE depth_quality, float map_res, float map_range,
                     sl::MeshFilterParameters::MESH_FILTER filter) noexcept;

        ~MonkeyVision() noexcept;

        void run(float marker_size, bool display, RuntimeErrorFlags *error_codes) noexcept;

        bool get_aruco(uint16_t id, ArucoData *data) noexcept;

        bool get_imu(ZedImuData *data) noexcept;

        bool get_frame(cv::Mat *image) noexcept;

        uint32_t frame_count() noexcept;

        void update_map() noexcept;

    private:
        static ZedStatusCode wrap_error_code(sl::ERROR_CODE error_code);

        // ZED camera handler
        sl::Camera zed;

        //
        cv::Mat image;

        bool has_image = false;

        //ZED spatial mapping mesh
        std::string mesh_file_path;

        sl::Mesh map_mesh;

        sl::MeshFilterParameters::MESH_FILTER mesh_filter;

        bool update_map_mesh = false;

        // Vectors for storing detected Aruco marker IDs and positional data
        std::vector<uint16_t> detected_ids;

        std::vector<ArucoData> detected_markers;

        // Struct for storing ZED IMU data
        ZedImuData zed_imu_data;

        bool imu_valid = false;

        // Struct for storing the latest camera capture
        ByteBufferShare camera_buffer;

        // Frame counter for periodic tasks
        uint32_t frame_counter = 0;

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
extern "C" bool get_aruco_data(uint16_t aruco_id, ArucoData *data, visual_processing::MonkeyVision *vision);

/**
 * @brief Get acceleration, orientation, and position data from the ZED IMU sensor.
 * @return Structure containing data from the ZED IMU.
 */
extern "C" bool get_zed_imu_data(ZedImuData *data, visual_processing::MonkeyVision *vision);

/**
 * @brief Get the frame buffer from the last camera capture.
 * @return Structure containing a byte array of the image pixels and integers for the image dimensions, which will default to 0 if the camera image could not be captured. 
 */
extern "C" bool get_camera_frame(visual_processing::MonkeyVision *vision, ByteBufferShare *image_buffer);

/**
 * @brief Set a flag to update the map mesh file the next time run_visual_processing() is called.
 *        NOTE: only update the mesh periodically as it is resource-intensive to do so.
 */
extern "C" void request_map_update(visual_processing::MonkeyVision *vision);

/**
 * @brief Get the number of frames that have been processed.
 * @return Integer count of the number of times run_visual_processing() has been called.
 */
extern "C" uint32_t get_frame_count(visual_processing::MonkeyVision *vision);

/**
 * @brief Intialize camera
 * @param mesh_path C-string path to save the stereo-scanned 3D environment mesh to.
 * @return Boolean value indicating whether the ZED camera was successfully initialized.
 */
extern "C" visual_processing::MonkeyVision *
visual_processing_init(const char *mesh_path, InitErrorFlags *init_flags, ZedCameraResolution camera_res, ZedDepthQuality depth_quality, ZedMappingResolution map_res, ZedMappingRange range,
                       ZedMeshFilter mesh_filter);

/**
 * @brief Run visual processing on a single video frame from the ZED camera.
 * @param marker_size Width of the Aruco marker in meters.
 * @param display Boolean value indicating if the function should display the camera view in a GUI window.
 * @return Boolean value indicating whether a camera frame could be successfully captured. 
 */
extern "C" void run_visual_processing(float marker_size, bool display, RuntimeErrorFlags *runtime_flags, visual_processing::MonkeyVision *vision);

/**
 * @brief Deallocate all dynamic memory and close ZED camera.
 */
extern "C" void visual_processing_dealloc(visual_processing::MonkeyVision *vision);
