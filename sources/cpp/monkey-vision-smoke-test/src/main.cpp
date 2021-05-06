#pragma once

#include <iostream>
#include <monkey_vision.h>

#define _DEBUG_

//Demo program
int main(int argc, char **argv)
{
    std::string path = "./mesh";
    bool initialized;
    visual_processing::MonkeyVision *vision = visual_processing_init(path.c_str(), &initialized, ZedCameraResolution::ResVGA100, ZedDepthQuality::DepthULTRA, ZedMappingResolution::MapMEDIUMRES, ZedMappingRange::MapNEAR, ZedMeshFilter::FilterMEDIUM);
    if (!initialized)
    {
        std::cout << "ERROR: Could not initialize ZED camera." << std::endl;
        visual_processing_dealloc(vision);
        return -1;
    }
    std::cout << "Loaded dictionary: Aruco Original." << std::endl;
    // Loop until 'q' is pressed
    char key = '.';
    while (key != 'q')
    {
        auto frame_count = get_frame_count(vision);
        bool mapping_available;
        bool camera_success = run_visual_processing(0.04, true, &mapping_available, vision);
        if (!camera_success)
        {
            std::cout << "ERROR: Could not capture frame from ZED camera." << std::endl;
            visual_processing_dealloc(vision);
            return -2;
        }
        if (frame_count % 60 == 0 && frame_count > 0) {
            std::cout << "Frame #" << frame_count << ", updating mesh..." << std::endl;
            if (mapping_available)
            {
                request_map_update(vision);
            }
            else
            {
                std::cout << "ERROR: Mapping unavailable" << std::endl;
            }
#ifdef _DEBUG_
            ArucoData marker;
            bool detected = get_aruco_data(69, &marker, vision);
            ZedImuData imu_data;
            get_zed_imu_data(&imu_data, vision);

            if (detected) {
                std::cout << "Aruco Marker 69: X=" << marker.x_dist << "m, Z=" << marker.z_dist << "m, Angle = "
                          << marker.y_rot*10 << "rad" << std::endl;
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

