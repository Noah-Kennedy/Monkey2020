#pragma once

#include <iostream>
#include <monkey_vision.h>

//Demo program
int main(int argc, char **argv)
{
    std::string path = "./mesh";
    visual_processing_init(path.c_str());
    std::cout << "Loaded dictionary: Aruco Original." << std::endl;
    // Loop until 'q' is pressed
    char key = '.';
    auto frame_count = get_frame_count();

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

