#include <iostream>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include "aruco.hpp"
#include "visual_processing_c.h"

using namespace std;
using namespace cv;
using namespace sl;

//ZED camera handler
Camera zed;
//Aruco dictionary used for detection - Aruco Original
auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

//Convert OpenCV matrix object to ZED matrix object
Mat slMat2cvMat(sl::Mat& input);
//Get corresponding depth measurement at specified camera pixel coordinate
float get_pixel_depth(int x, int y, sl::Mat img);

Mat slMat2cvMat(sl::Mat& input) {
    // Since Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // Mat and sl::Mat will share a single memory structure
    return Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

float get_pixel_depth(int x, int y, sl::Mat img) {
    //Convert image pixel to 3d point from point cloud
    sl::float4 point3d;
    img.getValue(x, y, &point3d);
    //Calculate absolute point depth from X,Y,Z
    float depth = sqrt(point3d.x*point3d.x + point3d.y*point3d.y + point3d.z*point3d.z);
    return depth;
}

bool visual_processing_init(zed_resolution camera_res, zed_depth_quality depth) {
    InitParameters init_params;
    //Select ZED camera resolution and FPS from argument
    switch (camera_res) {
        case 2K:
            init_params.camera_resolution = RESOLUTION::HD2K;
            init_params.camera_fps = 15;
            break;
        case 1080P:
            init_params.camera_resolution = RESOLUTION::HD1080;
            init_params.camera_fps = 30;
            break;
        case 720P:
            init_params.camera_resolution = RESOLUTION::HD720;
            init_params.camera_fps = 60;
            break;
        default:
            init_params.camera_resolution = RESOLUTION::VGA;
            init_params.camera_fps = 30;
    }
    //Select ZED camera depth mode from argument
    switch (depth) {
        case PERFORMANCE_DEPTH:
            init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
            break;
        case QUALITY_DEPTH:
            init_params.depth_mode = DEPTH_MODE::QUALITY;
            break;
        case ULTRA_DEPTH:
            init_params.depth_mode = DEPTH_MODE::ULTRA;
            break;
        default:
            init_params.depth_mode = DEPTH_MODE::NONE;
    }
    init_params.coordinate_units = UNIT::METER;
    //Initialize camera using given parameters
    auto returned_state = zed.open(init_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << "." << endl;
        return false;
    }
    return true;
}

bool run_visual_processing(aruco_data* aruco_list, bool display, float marker_size) {
    aruco_delete(aruco_list);

    auto cameraInfo = zed.getCameraInformation();
    Resolution image_size = cameraInfo.camera_resolution;
    sl::Mat image_zed(image_size, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(MEM::CPU));
    cv::Mat image_ocv_rgb;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    Matx33d camera_matrix = Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;

    Matx<float, 4, 1> dist_coeffs = Vec4f::zeros();

    cout << "Make sure the ArUco marker is an Aruco Original, measuring " << marker_size * 1000 << " mm" << endl;

    Transform pose;
    Pose zed_pose;
    vector<Vec3d> rvecs, tvecs;
    vector<int> ids;
    vector<vector<Point2f> > corners;
    string position_txt;


    if (zed.grab() == ERROR_CODE::SUCCESS) {
        // Retrieve the left image
        zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, image_size);

        // convert to RGB
        cvtColor(image_ocv, image_ocv_rgb, COLOR_RGBA2RGB);
        // detect marker
        aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);

        // get actual ZED position
        zed.getPosition(zed_pose);

        // display ZED position
        rectangle(image_ocv_rgb, Point(0, 0), Point(490, 75), Scalar(0, 0, 0), -1);
        putText(image_ocv_rgb, "Loaded dictionary : Aruco Original.     Press 'SPACE' to reset the camera position", Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(220, 220, 220));

        // if at least one marker detected
        if (ids.size() > 0) {
            aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
            pose.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));
            pose.setRotationVector(sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));
            pose.inverse();

            aruco::drawDetectedMarkers(image_ocv_rgb, corners, ids);
            aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size * 0.5f);
            position_txt = "Aruco x: " + to_string(pose.tx) + "; y: " + to_string(pose.ty) + "; z: " + to_string(pose.tz);
            putText(image_ocv_rgb, position_txt, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(124, 252, 124));

        }

        // Display image
        imshow("Image", image_ocv_rgb);
        
        return true;
    }
    return false;
}

void aruco_delete(aruco_data* head) {
    aruco_data* current_node = head;
    aruco_data* next_node;
    while (current_node != NULL) {
        next_node = current_node->next;
        free(current_node);
        current_node = next_node;
    }
}

//Demo program
int main(int argc, char **argv) {
    zed_resolution res = USB2;
    zed_depth_quality depth = ULTRA_DEPTH;
    visual_processing_init(res, depth);
    aruco_data* detected_markers;
    // Loop until 'q' is pressed
    char key = '.';
    while (key != 'q') {
        run_visual_processing(detected_markers, true);
        key = waitKey(10);
    }
    zed.close();
    return 0;
}

