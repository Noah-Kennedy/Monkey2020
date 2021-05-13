#pragma once

#include <iostream>
#include <monkey_vision.h>
#include <capture.h>

#define _DEBUG_

//Demo program
int main(int argc, char **argv)
{
    std::string path = "./mesh";
    InitErrorFlags init_flags;
    RuntimeErrorFlags loop_flags;
    visual_processing::MonkeyVision *vision = visual_processing_init(path.c_str(), &init_flags, ZedCameraResolution::ResVGA60, ZedDepthQuality::DepthQuality, ZedMappingResolution::MapMediumRes,
                                                                     ZedMappingRange::MapNear, ZedMeshFilter::FilterMedium);

    TimerData cameralot_timer;
    ByteBufferShare *cameralot_buffer;


    if (init_flags.map_status_code != ZedStatusCode::ZedErrorSuccess) {
        std::cout << "ERROR: Could not initialize ZED camera." << std::endl;
        visual_processing_dealloc(vision);
        return -1;
    }
    std::cout << "Loaded dictionary: Aruco Original." << std::endl;
    // Loop until 'q' is pressed
    char key = '.';
    while (key != 'q') {
        ReadStatus cameralot_status = vision->read(640, 480, "", cameralot_timer, cameralot_buffer);
        std::vector<uchar> decode_buffer;
        for (int i = 0; i < cameralot_buffer->length; ++i) {
            decode_buffer.push_back(cameralot_buffer->buffer[i]);
        }
        cv::Mat decoded_cameralot_frame = cv::imdecode(decode_buffer, cv::IMREAD_COLOR);
        cv::imshow("ZED Cameralot", decoded_cameralot_frame);

        auto frame_count = get_frame_count(vision);
        ZedStatusCode camera_code, imu_code;
        ZedSpatialMappingState map_state;
        run_visual_processing(0.04, false, &loop_flags, vision);
        if (frame_count % 60 == 0 && frame_count > 0) {
            std::cout << "Frame #" << frame_count << ", updating mesh..." << std::endl;
            if (loop_flags.map_status_code == ZedSpatialMappingState::ZedMapOk) {
                request_map_update(vision);
            } else {
                std::cout << "ERROR: Mapping unavailable" << std::endl;
            }
#ifdef _DEBUG_
            ArucoData marker;
            bool detected = get_aruco_data(69, &marker, vision);
            ZedImuData imu_data;
            get_zed_imu_data(&imu_data, vision);

            if (detected) {
                std::cout << "Aruco Marker 69: X=" << marker.x_dist << "m, Z=" << marker.z_dist << "m, Angle = "
                          << marker.y_rot * 10 << "rad" << std::endl;
            } else {
                std::cout << "Could not detect Aruco Marker 69" << std::endl;
            }
            std::cout << "IMU: X=" << imu_data.x_rot << "deg, Y=" << imu_data.y_rot << "deg, Z=" << imu_data.z_rot
                      << "deg" << std::endl;
#endif
        }

        key = cv::waitKey(10);
    }
    visual_processing_dealloc(vision);
    return 0;
}

