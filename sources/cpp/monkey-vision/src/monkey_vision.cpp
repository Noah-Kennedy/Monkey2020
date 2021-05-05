#include <iostream>

#include "monkey_vision.h"

/**************************************************************************************************
 * C++
 *************************************************************************************************/

visual_processing::MonkeyVision::MonkeyVision(bool *success)
{
    *success = true;
    // Initialize empty map mesh
    this.mesh_file_path = mesh_path;
    this.map_mesh.save(mesh_file_path, sl::MESH_FILE_FORMAT::PLY);
    //Set ZED camera parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = ZedCameraResolution;
    init_params.camera_fps = ZedCameraFps;
    init_params.depth_mode = ZedDepthQuality;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    // Initialize camera using given parameters
    auto returned_state = this.zed.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << "." << std::endl;
        *success = false;
        return;
    }
    // Enable IMU position tracking
    sl::PositionalTrackingParameters tracking_params;
    returned_state = this.zed.enablePositionalTracking(tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        *success = false;
        return;
    }
    // Enable spatial mapping
    sl::SpatialMappingParameters mapping_params;
    mapping_params.map_type = ZedMapType;
    mapping_params.resolution_meter = ZedMappingResolution;
    mapping_params.range_meter = ZedMappingRange;
    returned_state = this.zed.enableSpatialMapping(mapping_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        *success = false;
    }
}

visual_processing::MonkeyVision::~MonkeyVision()
{
    this.zed.disableSpatialMapping();
    this.zed.disablePositionalTracking();
    this.zed.close();
}


bool visual_processing::MonkeyVision::run(float marker_size, bool display);
{
    //Setup camera frame capture
    auto cameraInfo = this.zed.getCameraInformation();
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

    // Capture a camera frame
    if (this.zed.grab() == sl::ERROR_CODE::SUCCESS && this.zed.getSpatialMappingState() == sl::SPATIAL_MAPPING_STATE::OK)
    {
        // Reset data for new frame
        this.detected_ids.clear();
        this.detected_markers.clear();
        this.zed_imu_data = {0,0,0,0,0,0};

        // Retrieve the left image
        this.zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
        // Convert to RGB
        cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
        // Detect markers
        cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, this.detected_ids);
        // If at least one marker detected
        if (this.detected_ids.size() > 0)
        {
            //calculate pose for all detected markers
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
            //Save pose data into struct wrapper
            for (auto it = this.detected_ids.begin(); it != this.detected_ids.end(); ++it)
            {
                int index = it - this.detected_ids.begin();
                ArucoData marker_data;
                marker_data.x_dist = tvecs[index](0);
                marker_data.y_dist = tvecs[index](1);
                marker_data.z_dist = tvecs[index](2);
                marker_data.x_rot = rvecs[index](0);
                marker_data.y_rot = rvecs[index](1);
                marker_data.z_rot = rvecs[index](2);
                this.detected_markers.push_back(marker_data);
            }
            // Draw on image to highlight detected markers
            cv::aruco::drawDetectedMarkers(image_ocv_rgb, corners, this.detected_ids);
            cv::aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size * 1.5f);
        }
        // Display image
        if (display)
        {
            imshow("ZED Camera Feed", image_ocv_rgb);
        }

        // Read IMU data
        sl::SensorsData sensor_data;
        this.zed.getSensorsData(sensor_data, sl::TIME_REFERENCE::IMAGE);
        sl::float3 linear_accel = sensor_data.imu.linear_acceleration;
        sl::float3 angular_vel = sensor_data.imu.angular_velocity;
        sl::float3 translation = sensor_data.imu.pose.getTranslation();
        sl::float3 orientation = sensor_data.imu.pose.getOrientation();
        // Store IMU data
        this.zed_imu_data.x_accel = linear_accel[0];
        this.zed_imu_data.y_accel = linear_accel[1];
        this.zed_imu_data.z_accel = linear_accel[2];
        this.zed_imu_data.x_rot_vel = angular_vel[0];
        this.zed_imu_data.y_rot_vel = angular_vel[1];
        this.zed_imu_data.z_rot_vel = angular_vel[2];
        this.zed_imu_data.x_pos = translation[0];
        this.zed_imu_data.y_pos = translation[1];
        this.zed_imu_data.z_pos = translation[2];
        this.zed_imu_data.x_rot = orientation[0];
        this.zed_imu_data.y_rot = orientation[1];
        this.zed_imu_data.z_rot = orientation[2];

        // Update spatial map mesh every 30 frames (only done periodically because resource-intensive)
        if (this.update_map_mesh)
        {
            this.zed.requestSpatialMapAsync();
            this.update_map_mesh = false;
        }
        // Retrieve, filter, and save updated map when ready
        if (this.zed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::SUCCESS)
        {
            this.zed.retrieveSpatialMapAsync(this.map_mesh);
            this.map_mesh.filter(ZedMeshFilter);
            this.map_mesh.save(this.mesh_file_path, sl::MESH_FILE_FORMAT::PLY);
        }

        this.frame_count++;
        return true;
    }
    return false;
}

bool visual_processing::MonkeyVision::get_aruco(int id, ArucoData *data)
{
    data = NULL;
    // Check if any markers were detected
    if (this.detected_ids.size() > 0)
    {
        // Determine if the selected marker ID was detected
        auto iterator = find(this.detected_ids.begin(), this.detected_ids.end(), id);
        if (iterator != this.detected_ids.end()) 
        {
            // Return data for the Aruco marker which encodes the given ID
            int index = iterator - this.detected_ids.begin();
            *data = this.detected_markers[index];
            return true
        }
    }
    return false;
}

bool visual_processing::MonkeyVision::get_imu(ZedImuData *data)
{
    *data = this.zed_imu_data;
    return true;
}

bool visual_processing::MonkeyVision::get_frame(FrameBuffer* frame)
{
    // Not implemented
    return false;
}

long visual_processing::MonkeyVision::frame_count()
{
    return this.frame_count;
}

void visual_processing::MonkeyVision::update_map()
{
    this.update_map_mesh = true;
}

/**************************************************************************************************
 * C
 *************************************************************************************************/

bool get_aruco_data(int aruco_id, ArucoData *data, visual_processing:MonkeyVision *vision)
{
    return vision->get_aruco(aruco_id, data);
}

bool get_zed_imu_data(ZedImuData *data, visual_processing::MonkeyVision *vision)
{
    return vision->get_imu(data);
}

bool get_camera_frame(FrameBuffer *img, visual_processing::MonkeyVision *vision)
{
    return vision->get_frame(img);
}

void request_map_update(visual_processing::MonkeyVision *vision)
{
    vision->update_map();
}

long get_frame_count()
{
    return frame_count;
}

visual_processing::MonkeyVision* visual_processing_init(const char *mesh_path, bool *success) 
{
    return new visual_processing::MonkeyVision(mesh_path, success);
}

bool run_visual_processing(float marker_size, bool display, visual_processing::MonkeyVision* vision)
{
    return vision->run(marker_size, display);
}

void visual_processing_dealloc(visual_processing::MonkeyVision* vision)
{
    delete(vision);
}

//Demo program
int main(int argc, char **argv)
{
    std::string path = "./mesh";
    visual_processing_init(path.c_str()); 
    std::cout << "Loaded dictionary: Aruco Original." << std::endl;
    // Loop until 'q' is pressed
    char key = '.';
    while (key != 'q')
    {
        if (frame_count % 60 == 0 && frame_count > 0)
        {
            request_map_update();
            std::cout << "Frame #" << frame_count << ", updating mesh..." << std::endl;
#ifdef _DEBUG_
            ArucoData marker = get_aruco_data(69);
            ZedImuData imu_data = get_zed_imu_data();
            std::cout << "Aruco Marker 69: X=" << marker.x_dist << "m, Z=" << marker.z_dist << "m, Angle = " << marker.y_rot << "deg" << std::endl;
            std::cout << "IMU: X=" << imu_data.x_rot << "deg, Y=" << imu_data.y_rot << "deg, Z=" << imu_data.z_rot << "deg" << std::endl;
#endif 
        }
        run_visual_processing(0.10, true);
        key = cv::waitKey(10);
    }
    visual_processing_dealloc();
    return 0;
}

