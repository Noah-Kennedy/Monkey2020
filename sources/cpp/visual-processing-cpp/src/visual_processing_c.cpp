#include "visual_processing_c.h"
#include <sl/Camera.hpp>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <iostream>
#include <math.h>

//Width of the tags in meters
const float tag_size = 0.125;
//Camera focal length, in pixels
const float focal_length = 1000.0;

apriltag_family_t *tag_family;
apriltag_detector *tag_detector;

void visual_processing_init() {
    //Initialize AprilTag detector object
    tag_family = tag36h11_create();
    tag_detector = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector, tag_family);
}

zarray_t* detect_tags(cv::Mat img) {
    //Convert OpenCV image matrix into AprilTag image format
    image_u8_t img_header = {
        .width = img.cols,
        .height = img.rows,
        .stride = img.cols,
        .buf = img.data
    };
    //Run tag detector and generate zarray (resizable array) output
    zarray_t *detected_tags = apriltag_detector_detect(tag_detector, &img_header);
    return detected_tags;
}

void draw_boundary(cv::Mat img, apriltag_detection_t *tag) {
    //Draw boundary to highlight detected AprilTag on image capture
    line(img, cv::Point(tag->p[0][0], tag->p[0][1]), cv::Point(tag->p[1][0], tag->p[1][1]), cv::Scalar(0, 0xff, 0), 2);
    line(img, cv::Point(tag->p[0][0], tag->p[0][1]), cv::Point(tag->p[3][0], tag->p[3][1]), cv::Scalar(0, 0, 0xff), 2);
    line(img, cv::Point(tag->p[1][0], tag->p[1][1]), cv::Point(tag->p[2][0], tag->p[2][1]), cv::Scalar(0xff, 0, 0), 2);
    line(img, cv::Point(tag->p[2][0], tag->p[2][1]), cv::Point(tag->p[3][0], tag->p[3][1]), cv::Scalar(0xff, 0, 0), 2);
    //Display the ID text of the AprilTag
    std::stringstream ss;
    ss << tag->id;
    cv::String id_text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    float fontscale = 1.0;
    int baseline;
    cv::Size textsize = cv::getTextSize(id_text, fontface, fontscale, 2, &baseline);
    putText(img, id_text, cv::Point(tag->c[0]-textsize.width/2, tag->c[1]+textsize.height/2), fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
}

tag_data process_tag(apriltag_detection_t *tag, sl::Mat img) {
    tag_data data;
    //Get image coordinates of AprilTag
    data.x = tag->c[0];
    data.y = tag->c[1];
    //Get distance from camera
    data.distance = get_pixel_depth(data.x, data.y, img);
    //Get rotation of tag
    matd_t* rot = get_tag_pose(tag, tag_size, focal_length, focal_length, data.x, data.y);
    data.pitch = atan2(matd_get(rot, 3,2), matd_get(rot, 3,3));
    data.yaw = atan2(-matd_get(rot, 3,1), sqrt(matd_get(rot, 3,2)*matd_get(rot, 3,2) + matd_get(rot, 3,3)*matd_get(rot, 3,3)));
    data.roll = atan2(matd_get(rot, 2,1), matd_get(rot, 1,1));
    matd_destroy(rot);

    return data; 
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

cv::Mat slMat2cvMat(sl::Mat& input) {\
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    cv::cuda::GpuMat gpu_image = cv::cuda::GpuMat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::GPU), input.getStepBytes(sl::MEM::GPU));
    cv::Mat ocv_image;
    gpu_image.download(ocv_image);
    return ocv_image;
}

void visual_processing_dealloc() {
    apriltag_detector_destroy(tag_detector);
    tag36h11_destroy(tag_family);
}

int main() {
    sl::Camera zed;
    sl::Resolution image_res = zed.getCameraInformation().camera_resolution;
    sl::Mat zed_image(image_res.width/2, image_res.height/2, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
    cv::Mat opencv_image = slMat2cvMat(zed_image);
    visual_processing_init();
    
    int frame_count = 0;
    while (true) {
        frame_count++;
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            //Capture camera frame
            zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            //Find tags in image
            zarray_t* detected_tags = detect_tags(opencv_image);
            //Process all detected tags
            printf("Frame: %d\r\n", frame_count);
            if (zarray_size(detected_tags) == 0) {
                printf("No AprilTags detected in frame #%d\r\n", frame_count);
            }
            for (int i = 0; i < zarray_size(detected_tags); i++) {
                apriltag_detection_t *tag;
                zarray_get(detected_tags, i, &tag);
                draw_boundary(opencv_image, tag);
                tag_data tag_info = process_tag(tag, zed_image);
                printf("AprilTag ID: %d\r\n", tag->id);
                printf("Image coordinates: %d, %d\r\n", tag_info.x, tag_info.y);
                printf("Distance: %fm\r\n", tag_info.distance);
                printf("Pitch angle: %f\r\n", tag_info.pitch);
                printf("Yaw angle: %f\r\n", tag_info.yaw);
                printf("Roll angle: %f\r\n\r\n", tag_info.roll);
            }
            printf("\r\n");
            cv::imshow("Image", opencv_image);
            apriltag_detections_destroy(detected_tags);
        } else {
            printf("Error: Could not retrieve image from ZED. Exiting...\r\n");
            return 0;
        }
    }

    visual_processing_dealloc();
    return 0;
}

