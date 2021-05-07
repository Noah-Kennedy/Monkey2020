#pragma once

#include <opencv2/videoio.hpp>
#include <monkey-vision/monkey_vision.h>

/**************************************************************************************************
 * C
 *************************************************************************************************/
extern "C" enum ReadStatus
{
    Success = 0,
    NotOpen = 1,
    ReadFailed = 2,
    RetrieveFailed = 3,
    EmptyFrame = 4,
    EncodingFailed = 5,
};

extern "C" struct ByteBufferShare
{
    uchar *buffer;
    size_t length;
};

/**************************************************************************************************
 * C++
 *************************************************************************************************/

namespace cameralot {
    class CameraFeed
    {
    public:

        ReadStatus read(uint32_t width, uint32_t height, const char *ext) noexcept;

        bool get_buffer(ByteBufferShare *buffer) noexcept;


    private:
        visual_processing::MonkeyVision vision;

        std::vector<uchar> image_buffer;

        cv::Mat reading_frame;

        cv::Mat output_frame;

        bool has_image;


    public:
        CameraFeed(visual_processsing::MonkeyVision *vision) noexcept: image_buffer(), has_image(false) {this->vision = *vision}
    };
}

/**************************************************************************************************
 * C
 *************************************************************************************************/
extern "C" cameralot::CameraFeed *camera_feed_create(visual_processing::MonkeyVision *vision);
extern "C" void camera_feed_delete(cameralot::CameraFeed *feed);

extern "C" ReadStatus camera_feed_read(
        cameralot::CameraFeed *cameraFeed,
        uint32_t width,
        uint32_t height,
        char *ext
);

extern "C" bool camera_feed_get_buf(
        cameralot::CameraFeed *cameraFeed,
        ByteBufferShare *buf
);