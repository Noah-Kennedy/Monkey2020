#include <iostream>

#include "monkey_vision.h"

using namespace visual_processing;

/**************************************************************************************************
 * C++
 *************************************************************************************************/

MonkeyVision::MonkeyVision(std::string mesh_path, bool *success, sl::RESOLUTION camera_res, int fps, sl::DEPTH_MODE depth_quality, float map_res, float map_range, sl::MeshFilterParameters::MESH_FILTER filter)
{
    *success = true;
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
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << "." << std::endl;
        *success = false;
        return;
    }
    // Enable IMU position tracking
    sl::PositionalTrackingParameters tracking_params;
    returned_state = this->zed.enablePositionalTracking(tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        *success = false;
        return;
    }
    // Enable spatial mapping
    sl::SpatialMappingParameters mapping_params;
    mapping_params.map_type = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
    mapping_params.resolution_meter = this->mapping_resolution;
    mapping_params.range_meter = this->mapping_range;
    returned_state = this->zed.enableSpatialMapping(mapping_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        *success = false;
    }
}

MonkeyVision::~MonkeyVision()
{
    this->zed.disableSpatialMapping();
    this->zed.disablePositionalTracking();
    this->zed.close();
}

bool MonkeyVision::run(float marker_size, bool display, bool *mapping_available)
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
    if (this->zed.grab() == sl::ERROR_CODE::SUCCESS)
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
                int index = it - this->detected_ids.begin();
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
        this->zed.getSensorsData(sensor_data, sl::TIME_REFERENCE::IMAGE);
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
        this->imu_valid = true;

        if (this->zed.getSpatialMappingState() == sl::SPATIAL_MAPPING_STATE::OK)
        {
            *mapping_available = true;
        }
        else
        {
            *mapping_available = false;
        }
        // Update spatial map mesh every 30 frames (only done periodically because resource-intensive)
        if (this->update_map_mesh && *mapping_available)
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
        return true;
    }
    return false;
}

bool MonkeyVision::get_aruco(int id, ArucoData *data)
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
            int index = iterator - this->detected_ids.begin();
            *data = this->detected_markers[index];
            return true;
        }
    }
    return false;
}

bool MonkeyVision::get_imu(ZedImuData *data)
{
    *data = this->zed_imu_data;
    return this->imu_valid;
}

bool MonkeyVision::get_frame(FrameBuffer* frame)
{
    // Not implemented
    return false;
}

long MonkeyVision::frame_count()
{
    return this->frame_counter;
}

void MonkeyVision::update_map()
{
    this->update_map_mesh = true;
}

/**************************************************************************************************
 * C
 *************************************************************************************************/

bool get_aruco_data(int aruco_id, ArucoData *data, visual_processing::MonkeyVision *vision)
{
    return vision->get_aruco(aruco_id, data);
}

bool get_zed_imu_data(ZedImuData *data, MonkeyVision *vision)
{
    return vision->get_imu(data);
}

bool get_camera_frame(FrameBuffer *img, MonkeyVision *vision)
{
    return vision->get_frame(img);
}

void request_map_update(MonkeyVision *vision)
{
    vision->update_map();
}

long get_frame_count(MonkeyVision *vision)
{
    return vision->frame_count();
}

MonkeyVision* visual_processing_init(const char *mesh_path, bool *success, ZedCameraResolution camera_res, ZedDepthQuality depth_quality, ZedMappingResolution map_res, ZedMappingRange range, ZedMeshFilter mesh_filter)
{
    sl::RESOLUTION zed_resolution;
    int zed_fps;
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
            zed_fps = 100;
    }

    switch(depth_quality)
    {
        case DepthPERFORMANCE:
            zed_depth_quality = sl::DEPTH_MODE::PERFORMANCE;
            break;
        case DepthQUALITY:
            zed_depth_quality = sl::DEPTH_MODE::QUALITY;
            break;
        default:
            zed_depth_quality = sl::DEPTH_MODE::ULTRA;
    }

    switch(map_res)
    {
        case MapLOWRES:
            mapping_resolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION::LOW);
            break;
        case MapHIGHRES:
            mapping_resolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION::HIGH);
            break;
        default:
            mapping_resolution = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION::MEDIUM);
    }

    switch(range)
    {
        case MapNEAR:
            mapping_range = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::SHORT);
            break;
        case MapFAR:
            mapping_range = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::LONG);
            break;
        default:
            mapping_range = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
    }

    switch(mesh_filter)
    {
        case FilterLOW:
            mesh_filter_level = sl::MeshFilterParameters::MESH_FILTER::LOW;
            break;
        case FilterHIGH:
            mesh_filter_level = sl::MeshFilterParameters::MESH_FILTER::HIGH;
            break;
        default:
            mesh_filter_level = sl::MeshFilterParameters::MESH_FILTER::MEDIUM;
    }

    return new MonkeyVision(mesh_path, success, zed_resolution, zed_fps, zed_depth_quality, mapping_resolution, mapping_range, mesh_filter_level);
}

bool run_visual_processing(float marker_size, bool display, bool *mapping_success, MonkeyVision* vision)
{
    return vision->run(marker_size, display, mapping_success);
}

void visual_processing_dealloc(MonkeyVision* vision)
{
    delete(vision);
}
