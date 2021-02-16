#include <iostream>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
}

using namespace std;
using namespace cv;
using namespace sl;

int getOCVtype(sl::MAT_TYPE type);
cv::Mat slMat2cvMat(sl::Mat& input);
float get_pixel_depth(int x, int y, sl::Mat img);
matd_t* get_tag_pose(apriltag_detection_t *tag, float tag_size, float fx, float fy, float cx, float cy);

int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

cv::Mat slMat2cvMat(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

float get_pixel_depth(int x, int y, sl::Mat img) {
    //Convert image pixel to 3d point from point cloud
    sl::float4 point3d;
    img.getValue(x, y, &point3d);
    //Calculate absolute point depth from X,Y,Z
    float depth = sqrt(point3d.x*point3d.x + point3d.y*point3d.y + point3d.z*point3d.z);
    return depth;
}

matd_t* get_tag_pose(apriltag_detection_t *tag, float tag_size, float fx, float fy, float cx, float cy) {
    //Initialize tag data
    apriltag_detection_info_t info;
    info.det = tag;
    info.tagsize = tag_size;
    info.fx = fx;
    info.fy = fy;
    info.cx = cx;
    info.cy = cy;
    //Estimate pose using homography
    apriltag_pose_t pose;
    estimate_tag_pose(&info, &pose);
    //Return rotation matrix
    return pose.R;
}

int main(int argc, char *argv[])
{
    // Initialize camera
   /* VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }*/
    Camera zed;
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::VGA; // Use VGA video mode
    init_params.camera_fps = 30; // Set fps at 30
    init_params.depth_mode = DEPTH_MODE::ULTRA;
    init_params.coordinate_units = UNIT::METER;
    auto returned_state = zed.open(init_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << ", exit program." << endl;
        return EXIT_FAILURE;
    }
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    Resolution image_size = zed.getCameraInformation().camera_resolution;
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;
    Resolution new_image_size(new_width, new_height);
    sl::Mat frame_zed(new_width, new_height, MAT_TYPE::U8_C4);
    sl::Mat point_cloud;

    // Initialize tag detector
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    cv::Mat frame, gray, image_depth;
    sl::Mat frame_depth;
    while (true) {
        //cap >> frame;
        zed.grab();
        zed.retrieveImage(frame_zed, VIEW::LEFT);
        zed.retrieveImage(frame_depth, VIEW::DEPTH);
        zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);
        frame = slMat2cvMat(frame_zed);
        image_depth = slMat2cvMat(frame_depth);
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        // Process detections
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            //Draw borders
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            //Get distance
            float distance = get_pixel_depth(det->c[0], det->c[1], point_cloud);
            //Get angle
            //matd_t* rot = get_tag_pose(det, 0.125, 1000, 1000, det->c[0], det->c[1]);
            //float angle = atan2(-matd_get(rot, 3,1), sqrt(matd_get(rot, 3,2)*matd_get(rot, 3,2) + matd_get(rot, 3,3)*matd_get(rot, 3,3)));

            //Display distance
            stringstream ss;
            ss << setprecision(2) << distance << "m";
            cv::String text = ss.str();
            int fontface = FONT_HERSHEY_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
            if (text != "nanm" && text != "infm") {
                putText(frame, text, Point(det->c[0]+textsize.width/2, det->c[1]-textsize.height/2), fontface, fontscale, Scalar(0, 0, 0), 6);
                putText(frame, text, Point(det->c[0]+textsize.width/2, det->c[1]-textsize.height/2), fontface, fontscale, Scalar(0xff, 0xff, 0xff), 2);
            }
            //Display angle
            //ss.str("");
            //ss.clear();
            //ss << setprecision(2) << angle << "deg";
            //text = ss.str();
            //textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
            //putText(frame, text, Point(det->c[0]+textsize.width, det->c[1]+textsize.height), fontface, fontscale, Scalar(0, 0, 0), 6);
            //putText(frame, text, Point(det->c[0]+textsize.width, det->c[1]+textsize.height), fontface, fontscale, Scalar(0xff, 0xff, 0xff), 2);
        }
        apriltag_detections_destroy(detections);
        cv::Mat overlay;
        addWeighted(image_depth, 0, frame, 1, 0, overlay);
        cv::Mat output;
        cv::resize(overlay, output, cv::Size(frame.size().width*2.5, frame.size().height*2.5));
        imshow("Tag Detections", output);
        if (waitKey(30) >= 0)
            break;
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}
