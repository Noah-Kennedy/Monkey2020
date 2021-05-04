#include <iostream>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include "aruco.hpp"
#include "monkey_vision.h"

// ZED camera handler
sl::Camera zed;
// Vectors for storing detected Aruco marker IDs and positional data
std::vector<int> detected_ids;
std::vector<ArucoData> detected_markers;
// Struct for storing ZED IMU data
ZedImuData zed_imu_data;
// Struct for storing the latest camera capture
FrameBuffer camera_frame;

// Aruco dictionary used for detection - Aruco Original
const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

// Get corresponding OpenCV matrix subtype for a given ZED library matrix subtype
int getOCVtype(sl::MAT_TYPE type);
// Convert ZED matrix object to OpenCV matrix object
// TODO pls check is ambiguous
cv::Mat slMat2cvMat(sl::Mat &input);
// Get corresponding depth measurement at specified camera pixel coordinate
float get_pixel_depth(int x, int y, sl::Mat img);

int getOCVtype(sl::MAT_TYPE type)
{
    int cv_type = -1;
    switch (type)
    {
        case sl::MAT_TYPE::F32_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }
    return cv_type;
}

cv::Mat slMat2cvMat(sl::Mat &input)
{
    // Since Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

float get_pixel_depth(int x, int y, sl::Mat img)
{
    // Convert image pixel to 3d point from point cloud
    sl::float4 point3d;
    img.getValue(x, y, &point3d);
    // Calculate absolute point depth from X,Y,Z
    float depth = sqrt(point3d.x * point3d.x + point3d.y * point3d.y + point3d.z * point3d.z);
    return depth;
}

bool visual_processing_init(ZedResolution camera_res, ZedDepthQuality depth)
{
    sl::InitParameters init_params;
    // Select ZED camera resolution and FPS from argument
    switch (camera_res)
    {
        case ZedResolution::Res2K:
            init_params.camera_resolution = sl::RESOLUTION::HD2K;
            init_params.camera_fps = 15;
            break;
        case ZedResolution::Res1080P:
            init_params.camera_resolution = sl::RESOLUTION::HD1080;
            init_params.camera_fps = 30;
            break;
        case ZedResolution::Res720P:
            init_params.camera_resolution = sl::RESOLUTION::HD720;
            init_params.camera_fps = 60;
            break;
        default:
            init_params.camera_resolution = sl::RESOLUTION::VGA;
            init_params.camera_fps = 30;
    }
    // Select ZED camera depth mode from argument
    switch (depth)
    {
        case PERFORMANCE_DEPTH:
            init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
            break;
        case QUALITY_DEPTH:
            init_params.depth_mode = sl::DEPTH_MODE::QUALITY;
            break;
        case ULTRA_DEPTH:
            init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
            break;
        default:
            init_params.depth_mode = sl::DEPTH_MODE::NONE;
    }
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    // Initialize camera using given parameters
    auto returned_state = zed.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << "." << std::endl;
        return false;
    }
    std::cout << "Loaded dictionary: Aruco Original." << std::endl;
    return true;
}

ArucoData get_aruco_data(int aruco_id)
{
    // Check if any markers were detected
    if (detected_ids.size() > 0)
    {
        // Determine if the selected marker ID was detected
        auto iterator = find(detected_ids.begin(), detected_ids.end(), aruco_id);
        if (iterator != detected_ids.end()) 
        {
            // Return data for the Aruco marker which encodes the given ID
            int index = iterator - detected_ids.begin();
            return detected_markers[index];
        }
    }
    // If an Aruco with the given ID has not been detected, return a default ArucoData struct with -1 for all member values
    ArucoData default_no_data;
    return default_no_data;
}

ZedImuData get_zed_imu_data()
{
    return zed_imu_data;
}

FrameBuffer get_camera_frame()
{
    return camera_frame;
}

bool run_visual_processing(float marker_size, const char *mesh_path, bool display)
{
    auto cameraInfo = zed.getCameraInformation();
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

    sl::Pose zed_pose;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> corners;

    if (zed.grab() == sl::ERROR_CODE::SUCCESS)
    {
        // Reset data for new frame
        detected_ids.clear();
        detected_markers.clear();
        zed_imu_data = {-1,-1,-1,-1,-1,-1};

        // Retrieve the left image
        zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);

        // Convert to RGB
        cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
        // Detect markers
        cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, detected_ids);

        // If at least one marker detected
        if (detected_ids.size() > 0)
        {
            //calculate pose for all detected markers
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
            //Save pose data into struct wrapper
            for (auto it = detected_ids.begin(); it != detected_ids.end(); ++it)
            {
                int index = it - detected_ids.begin();
                ArucoData marker_data;
                marker_data.x_dist = tvecs[index](0);
                marker_data.y_dist = tvecs[index](1);
                marker_data.z_dist = tvecs[index](2);
                marker_data.x_rot = rvecs[index](0);
                marker_data.x_rot = rvecs[index](1);
                marker_data.x_rot = rvecs[index](2);
                detected_markers.push_back(marker_data);
            }
            // Draw on image to highlight detected markers
            cv::aruco::drawDetectedMarkers(image_ocv_rgb, corners, detected_ids);
            cv::aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size * 1.5f);
        }

        // Display image
        if (display)
        {
            imshow("Image", image_ocv_rgb);
        }
        return true;
    }
    return false;
}

void visual_processing_dealloc()
{
    zed.close();
}

//Demo program
int main(int argc, char **argv)
{
    visual_processing_init(ZedResolution::ResUSB2, ZedDepthQuality::ULTRA_DEPTH);
    // Loop until 'q' is pressed
    char key = '.';
    while (key != 'q')
    {
        std::string path = "./mesh.obj";
        run_visual_processing(0.10, path.c_str(), true);
        key = cv::waitKey(10);
    }
    visual_processing_dealloc();
    return 0;
}

