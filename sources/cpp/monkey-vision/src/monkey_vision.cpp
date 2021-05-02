#include <iostream>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include "aruco.hpp"
#include "monkey_vision.h"

//ZED camera handler
sl::Camera zed;

//Aruco dictionary used for detection - Aruco Original
auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

//Convert OpenCV matrix object to ZED matrix object
// TODO pls check is ambiguous
cv::Mat slMat2cvMat(sl::Mat &input);

//Get corresponding depth measurement at specified camera pixel coordinate
float get_pixel_depth(int x, int y, sl::Mat img);

int getOCVtype(sl::MAT_TYPE type)
{
    int cv_type = -1;
    switch (type) {
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
    //Convert image pixel to 3d point from point cloud
    sl::float4 point3d;
    img.getValue(x, y, &point3d);
    //Calculate absolute point depth from X,Y,Z
    float depth = sqrt(point3d.x * point3d.x + point3d.y * point3d.y + point3d.z * point3d.z);
    return depth;
}

bool visual_processing_init(ZedResolution camera_res, ZedDepthQuality depth)
{
    sl::InitParameters init_params;
    //Select ZED camera resolution and FPS from argument
    switch (camera_res) {
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
    //Select ZED camera depth mode from argument
    switch (depth) {
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
    //Initialize camera using given parameters
    auto returned_state = zed.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error " << returned_state << "." << std::endl;
        return false;
    }
    return true;
}

bool run_visual_processing(ArucoData *aruco_list, bool display, float marker_size)
{
    aruco_delete(aruco_list);

    auto cameraInfo = zed.getCameraInformation();
    sl::Resolution image_size = cameraInfo.camera_resolution;
    sl::Mat image_zed(image_size, sl::MAT_TYPE::U8_C4);
    cv::Mat image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));
    cv::Mat image_ocv_rgb;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;

    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    std::cout << "Make sure the ArUco marker is an Aruco Original, measuring " << marker_size * 1000 << " mm" << std::endl;

    sl::Transform pose;
    sl::Pose zed_pose;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::string position_txt;


    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
        // Retrieve the left image
        zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);

        // convert to RGB
        cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
        // detect marker
        cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);

        // get actual ZED position
        zed.getPosition(zed_pose);

        // display ZED position
        rectangle(image_ocv_rgb, cv::Point(0, 0), cv::Point(490, 75), cv::Scalar(0, 0, 0), -1);
        putText(image_ocv_rgb, "Loaded dictionary : Aruco Original.     Press 'SPACE' to reset the camera position", cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(220, 220, 220));

        // if at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
            pose.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));
            pose.setRotationVector(sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));
            pose.inverse();

            cv::aruco::drawDetectedMarkers(image_ocv_rgb, corners, ids);
            cv::aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size * 0.5f);
            position_txt = "Aruco x: " + std::to_string(pose.tx) + "; y: " + std::to_string(pose.ty) + "; z: " + std::to_string(pose.tz);
            putText(image_ocv_rgb, position_txt, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(124, 252, 124));

        }

        // Display image
        imshow("Image", image_ocv_rgb);

        return true;
    }
    return false;
}

void aruco_delete(ArucoData *head)
{
    ArucoData *current_node = head;
    ArucoData *next_node;
    while (current_node != NULL) {
        next_node = current_node->next;
        free(current_node);
        current_node = next_node;
    }
}

//Demo program
int main(int argc, char **argv)
{
    ZedResolution res = ZedResolution::ResUSB2;
    ZedDepthQuality depth = ULTRA_DEPTH;
    visual_processing_init(res, depth);
    ArucoData *detected_markers;
    // Loop until 'q' is pressed
    char key = '.';
    while (key != 'q') {
        // TODO fix this; I added NAN to make it compile, there was no third arg before
        run_visual_processing(detected_markers, true, NAN);
        key = cv::waitKey(10);
    }
    zed.close();
    return 0;
}

