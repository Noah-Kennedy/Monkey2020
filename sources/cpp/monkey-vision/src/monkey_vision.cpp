#include <iostream>

#include "monkey_vision.h"

using namespace visual_processing;

/**************************************************************************************************
 * C++
 *************************************************************************************************/

MonkeyVision::MonkeyVision(std::string mesh_path, InitErrorFlags *error_codes, sl::RESOLUTION camera_res, uint8_t fps, sl::DEPTH_MODE depth_quality, float map_res, float map_range, sl::MeshFilterParameters::MESH_FILTER filter) noexcept
{
    this->mapping_resolution = map_res;
    this->mapping_range = map_range;
    this->mesh_filter = filter;
    // Initialize empty map mesh
    this->mesh_file_path = mesh_path;
    this->map_mesh.save(mesh_file_path.c_str(), sl::MESH_FILE_FORMAT::PLY);
    //Set ZED camera parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = camera_res;
    init_params.camera_fps = fps;
    init_params.depth_mode = depth_quality;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    // Initialize camera using given parameters
    auto returned_state = this->zed.open(init_params);
    error_codes->camera_status_code = wrap_error_code(returned_state);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << ". Camera could not be initialized." << std::endl;
        error_codes->imu_status_code = ZedErrorCameraNotInitialized;
        error_codes->map_status_code = ZedErrorCameraNotInitialized;
        return;
    }
    this->is_open = true;
    // Enable IMU position tracking
    sl::PositionalTrackingParameters tracking_params;
    returned_state = this->zed.enablePositionalTracking(tracking_params);
    error_codes->imu_status_code = wrap_error_code(returned_state);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        this->zed.close();
        error_codes->map_status_code = ZedErrorFailure;
        return;
    }
    // Enable spatial mapping
    sl::SpatialMappingParameters mapping_params;
    mapping_params.map_type = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
    mapping_params.resolution_meter = this->mapping_resolution;
    mapping_params.range_meter = this->mapping_range;
    returned_state = this->zed.enableSpatialMapping(mapping_params);
    error_codes->map_status_code = wrap_error_code(returned_state);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        this->zed.disablePositionalTracking();
        this->zed.close();
    }
}

MonkeyVision::~MonkeyVision()
{
    this->zed.disableSpatialMapping();
    this->zed.disablePositionalTracking();
    this->zed.close();
}

void MonkeyVision::run(float marker_size, bool display, RuntimeErrorFlags *error_codes) noexcept
{
    this->imu_valid = false;
    //Setup camera frame capture
    auto cameraInfo = this->zed.getCameraInformation();
    sl::Resolution image_size = cameraInfo.camera_resolution;
    sl::Mat image_zed(image_size, sl::MAT_TYPE::U8_C4);
    cv::Mat image_ocv(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));
    cv::Mat image_ocv_rgb;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;

    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> corners;

    //Aruco marker dictionay
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // Capture a camera frame
    auto returned_state = this->zed.grab();
    error_codes->camera_status_code = wrap_error_code(returned_state);
    if (returned_state == sl::ERROR_CODE::SUCCESS)
    {
        // Reset data for new frame
        this->detected_ids.clear();
        this->detected_markers.clear();
        this->zed_imu_data = {0,0,0,0,0,0};

        // Retrieve the left image
        this->zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
        // Convert to RGB
        cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
        // Detect markers
        cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, this->detected_ids);
        // If at least one marker detected
        if (this->detected_ids.size() > 0)
        {
            //calculate pose for all detected markers
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
            //Save pose data into struct wrapper
            for (auto it = this->detected_ids.begin(); it != this->detected_ids.end(); ++it)
            {
                uint16_t index = it - this->detected_ids.begin();
                ArucoData marker_data;
                marker_data.x_dist = tvecs[index](0);
                marker_data.y_dist = tvecs[index](1);
                marker_data.z_dist = tvecs[index](2);
                marker_data.x_rot = rvecs[index](0);
                marker_data.y_rot = rvecs[index](1);
                marker_data.z_rot = rvecs[index](2);
                this->detected_markers.push_back(marker_data);
            }
            // Draw on image to highlight detected markers
            cv::aruco::drawDetectedMarkers(image_ocv_rgb, corners, this->detected_ids);
            cv::aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size * 1.5f);
        }
        // Display image
        if (display)
        {
            imshow("ZED Camera Feed", image_ocv_rgb);
        }

        // Read IMU data
        sl::SensorsData sensor_data;
        returned_state = this->zed.getSensorsData(sensor_data, sl::TIME_REFERENCE::IMAGE);
        error_codes->imu_status_code = wrap_error_code(returned_state);
        this->imu_valid = (returned_state == sl::ERROR_CODE::SUCCESS);
        sl::float3 linear_accel = sensor_data.imu.linear_acceleration;
        sl::float3 angular_vel = sensor_data.imu.angular_velocity;
        sl::float3 translation = sensor_data.imu.pose.getTranslation();
        sl::float3 orientation = sensor_data.imu.pose.getOrientation();
        // Store IMU data
        this->zed_imu_data.x_accel = linear_accel[0];
        this->zed_imu_data.y_accel = linear_accel[1];
        this->zed_imu_data.z_accel = linear_accel[2];
        this->zed_imu_data.x_rot_vel = angular_vel[0];
        this->zed_imu_data.y_rot_vel = angular_vel[1];
        this->zed_imu_data.z_rot_vel = angular_vel[2];
        this->zed_imu_data.x_pos = translation[0];
        this->zed_imu_data.y_pos = translation[1];
        this->zed_imu_data.z_pos = translation[2];
        this->zed_imu_data.x_rot = orientation[0];
        this->zed_imu_data.y_rot = orientation[1];
        this->zed_imu_data.z_rot = orientation[2];

        // Update spatial map mesh every 30 frames (only done periodically because resource-intensive)
        auto map_status = this->zed.getSpatialMappingState();
        switch (map_status)
        {
            case sl::SPATIAL_MAPPING_STATE::INITIALIZING:
                error_codes->map_status_code = ZedMapInitializing;
                break;
            case sl::SPATIAL_MAPPING_STATE::OK:
                error_codes->map_status_code = ZedMapOk;
                break;
            case sl::SPATIAL_MAPPING_STATE::NOT_ENOUGH_MEMORY:
                error_codes->map_status_code = ZedMapNotEnoughMemory;
                break;
            case sl::SPATIAL_MAPPING_STATE::NOT_ENABLED:
                error_codes->map_status_code = ZedMapNotEnabled;
                break;
            case sl::SPATIAL_MAPPING_STATE::FPS_TOO_LOW:
                error_codes->map_status_code = ZedMapFpsTooLow;
                break;
            default:
                NULL;
        }
        if (this->update_map_mesh && map_status == sl::SPATIAL_MAPPING_STATE::OK)
        {
            this->zed.requestSpatialMapAsync();
            this->update_map_mesh = false;
        }
        // Retrieve, filter, and save updated map when ready
        if (this->zed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::SUCCESS)
        {
            this->zed.retrieveSpatialMapAsync(this->map_mesh);
            this->map_mesh.filter(this->mesh_filter);
            this->map_mesh.save(this->mesh_file_path.c_str(), sl::MESH_FILE_FORMAT::PLY);
        }

        this->frame_counter++;
    }
}

bool MonkeyVision::get_aruco(uint16_t id, ArucoData *data) noexcept
{
    *data = {0,0,0,0,0,0};
    // Check if any markers were detected
    if (this->detected_ids.size() > 0)
    {
        // Determine if the selected marker ID was detected
        auto iterator = find(this->detected_ids.begin(), this->detected_ids.end(), id);
        if (iterator != this->detected_ids.end())
        {
            // Return data for the Aruco marker which encodes the given ID
            uint16_t index = iterator - this->detected_ids.begin();
            *data = this->detected_markers[index];
            return true;
        }
    }
    return false;
}

bool MonkeyVision::get_imu(ZedImuData *data) noexcept
{
    *data = this->zed_imu_data;
    return this->imu_valid;
}

uint32_t MonkeyVision::frame_count() noexcept
{
    return this->frame_counter;
}

void MonkeyVision::update_map() noexcept
{
    this->update_map_mesh = true;
}

bool MonkeyVision::is_opened() const noexcept
{
    return this->is_open;
}

ReadStatus MonkeyVision::read_frame(cv::Mat &frame, TimerData &td) noexcept
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    auto cameraInfo = this->zed.getCameraInformation();
    sl::Resolution image_size = cameraInfo.camera_resolution;
    sl::Mat image_zed(image_size, sl::MAT_TYPE::U8_C4);
    cv::Mat image_ocv(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));

    ReadStatus status = ReadStatus::Success;

    // start timer for grab
    auto t1 = high_resolution_clock::now();

    if (this->zed.grab() != sl::ERROR_CODE::SUCCESS) {
        status = ReadStatus::ReadFailed;
    }

    // record grab time
    auto t2 = high_resolution_clock::now();
    td.grab_millis = duration_cast<milliseconds>(t2 - t1).count();

    // start timer for retrieve
    t1 = high_resolution_clock::now();
    if (status == ReadStatus::Success && this->zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size) != sl::ERROR_CODE::SUCCESS) {
        status = ReadStatus::RetrieveFailed;
    }

    cvtColor(image_ocv, frame, cv::COLOR_RGBA2RGB);

    // record retrieve time
    t2 = high_resolution_clock::now();
    td.retrieve_millis = duration_cast<milliseconds>(t2 - t1).count();

    return status;
}

ZedStatusCode MonkeyVision::wrap_error_code(sl::ERROR_CODE error_code)
{
    switch (error_code) {
        case sl::ERROR_CODE::SUCCESS:
            return ZedErrorSuccess;
        case sl::ERROR_CODE::FAILURE:
            return ZedErrorFailure;
        case sl::ERROR_CODE::NO_GPU_COMPATIBLE:
            return ZedErrorNoGpuCompatible;
        case sl::ERROR_CODE::NOT_ENOUGH_GPU_MEMORY:
            return ZedErrorNotEnoughGpuMemory;
        case sl::ERROR_CODE::CAMERA_NOT_DETECTED:
            return ZedErrorCameraNotDetected;
        case sl::ERROR_CODE::SENSORS_NOT_AVAILABLE:
            return ZedErrorSensorsNotAvailable;
        case sl::ERROR_CODE::INVALID_RESOLUTION:
            return ZedErrorInvalidResolution;
        case sl::ERROR_CODE::LOW_USB_BANDWIDTH:
            return ZedErrorLowUsbBandwidth;
        case sl::ERROR_CODE::CALIBRATION_FILE_NOT_AVAILABLE:
            return ZedErrorCalibrationFileNotAvailable;
        case sl::ERROR_CODE::INVALID_CALIBRATION_FILE:
            return ZedErrorInvalidCalibrationFile;
        case sl::ERROR_CODE::INVALID_SVO_FILE:
            return ZedErrorInvalidSvoFile;
        case sl::ERROR_CODE::SVO_RECORDING_ERROR:
            return ZedErrorSvoRecordingError;
        case sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION:
            return ZedErrorSvoUnsupportedCompression;
        case sl::ERROR_CODE::END_OF_SVOFILE_REACHED:
            return ZedErrorEndOfSVOFileReached;
        case sl::ERROR_CODE::INVALID_COORDINATE_SYSTEM:
            return ZedErrorInvalidCoordinateSystem;
        case sl::ERROR_CODE::INVALID_FIRMWARE:
            return ZedErrorInvalidFirmware;
        case sl::ERROR_CODE::INVALID_FUNCTION_PARAMETERS:
            return ZedErrorInvalidFunctionParameters;
        case sl::ERROR_CODE::CUDA_ERROR:
            return ZedErrorCudaError;
        case sl::ERROR_CODE::CAMERA_NOT_INITIALIZED:
            return ZedErrorCameraNotInitialized;
        case sl::ERROR_CODE::NVIDIA_DRIVER_OUT_OF_DATE:
            return ZedErrorNvidiaDriverOutOfDate;
        case sl::ERROR_CODE::INVALID_FUNCTION_CALL:
            return ZedErrorInvalidFunctionCall;
        case sl::ERROR_CODE::CORRUPTED_SDK_INSTALLATION:
            return ZedErrorCorruptedSdkInstallation;
        case sl::ERROR_CODE::INCOMPATIBLE_SDK_VERSION:
            return ZedErrorIncompatibleSdkVersion;
        case sl::ERROR_CODE::INVALID_AREA_FILE:
            return ZedErrorInvalidAreaFile;
        case sl::ERROR_CODE::INCOMPATIBLE_AREA_FILE:
            return ZedErrorIncompatibleAreaFile;
        case sl::ERROR_CODE::CAMERA_FAILED_TO_SETUP:
            return ZedErrorCameraFailedToSetup;
        case sl::ERROR_CODE::CAMERA_DETECTION_ISSUE:
            return ZedErrorCameraDetectionIssue;
        case sl::ERROR_CODE::CANNOT_START_CAMERA_STREAM:
            return ZedErrorCannotStartCameraStream;
        case sl::ERROR_CODE::NO_GPU_DETECTED:
            return ZedErrorNoGpuDetected;
        case sl::ERROR_CODE::PLANE_NOT_FOUND:
            return ZedErrorPlaneNotFound;
        case sl::ERROR_CODE::MODULE_NOT_COMPATIBLE_WITH_CAMERA:
            return ZedErrorModuleNotCompatibleWithCamera;
        case sl::ERROR_CODE::MOTION_SENSORS_REQUIRED:
            return ZedErrorMotionSensorsRequired;
        default:
            ZedStatusCode none;
            return none;
    }
}

/**************************************************************************************************
 * C
 *************************************************************************************************/

bool get_aruco_data(uint16_t aruco_id, ArucoData *data, visual_processing::MonkeyVision *vision)
{
    return vision->get_aruco(aruco_id, data);
}

bool get_zed_imu_data(ZedImuData *data, MonkeyVision *vision)
{
    return vision->get_imu(data);
}

void request_map_update(MonkeyVision *vision)
{
    vision->update_map();
}

uint32_t get_frame_count(MonkeyVision *vision)
{
    return vision->frame_count();
}

MonkeyVision* visual_processing_init(const char *mesh_path, InitErrorFlags *init_flags, ZedCameraResolution camera_res, ZedDepthQuality depth_quality, ZedMappingResolution map_res, ZedMappingRange range, ZedMeshFilter mesh_filter)
{
    sl::RESOLUTION zed_resolution;
    uint8_t zed_fps;
    sl::DEPTH_MODE zed_depth_quality;
    float mapping_resolution, mapping_range;
    sl::MeshFilterParameters::MESH_FILTER mesh_filter_level;

    switch(camera_res)
    {
        case Res2K15:
            zed_resolution = sl::RESOLUTION::HD2K;
            zed_fps = 15;
            break;
        case Res1080HD30:
            zed_resolution = sl::RESOLUTION::HD1080;
            zed_fps = 30;
            break;
        case Res720HD60:
            zed_resolution = sl::RESOLUTION::HD720;
            zed_fps = 60;
            break;
        default:
            zed_resolution = sl::RESOLUTION::VGA;
            zed_fps = 60;
    }

    switch(depth_quality)
    {
        case DepthPerformance:
            zed_depth_quality = sl::DEPTH_MODE::PERFORMANCE;
            break;
        case DepthQuality:
            zed_depth_quality = sl::DEPTH_MODE::QUALITY;
            break;
        default:
            zed_depth_quality = sl::DEPTH_MODE::ULTRA;
    }

    switch(map_res)
    {
        case MapLowRes:
            mapping_resolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION::LOW);
            break;
        case MapHighRes:
            mapping_resolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION::HIGH);
            break;
        default:
            mapping_resolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION::MEDIUM);
    }

    switch(range)
    {
        case MapNear:
            mapping_range = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::SHORT);
            break;
        case MapFar:
            mapping_range = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::LONG);
            break;
        default:
            mapping_range = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
    }

    switch(mesh_filter)
    {
        case FilterLow:
            mesh_filter_level = sl::MeshFilterParameters::MESH_FILTER::LOW;
            break;
        case FilterHigh:
            mesh_filter_level = sl::MeshFilterParameters::MESH_FILTER::HIGH;
            break;
        default:
            mesh_filter_level = sl::MeshFilterParameters::MESH_FILTER::MEDIUM;
    }

    std::string path(mesh_path);
    MonkeyVision *vision = new MonkeyVision(path, init_flags, zed_resolution, zed_fps, zed_depth_quality, mapping_resolution, mapping_range, mesh_filter_level);
    return vision;
}

void run_visual_processing(float marker_size, bool display, RuntimeErrorFlags *runtime_flags, visual_processing::MonkeyVision *vision)
{
    vision->run(marker_size, display, runtime_flags);
}

void visual_processing_dealloc(MonkeyVision* vision)
{
    delete(vision);
}
