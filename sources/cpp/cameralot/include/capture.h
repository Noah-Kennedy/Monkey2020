#pragma once

#include <opencv2/videoio.hpp>

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

extern "C" struct TimerData
{
    uint32_t grab_millis;
    uint32_t retrieve_millis;
    uint32_t resize_millis;
    uint32_t encode_millis;
};

/**************************************************************************************************
 * C++
 *************************************************************************************************/

namespace cameralot {
    class CameraFeed
    {
    public:
        bool is_opened() const noexcept;

        bool open(int index, int api_preference = cv::CAP_V4L2) noexcept;

        ReadStatus read(uint32_t width, uint32_t height, const char *ext, TimerData *td) noexcept;

        bool get_buffer(ByteBufferShare *buffer) noexcept;


    private:
        cv::VideoCapture videoCapture;

        std::vector<uchar> image_buffer;

        cv::Mat reading_frame;

        cv::Mat output_frame;

        bool has_image;


    public:
        CameraFeed() noexcept: videoCapture(), image_buffer(), has_image(false) {}
    };
}

/**************************************************************************************************
 * C
 *************************************************************************************************/
extern "C" cameralot::CameraFeed *camera_feed_create();
extern "C" void camera_feed_delete(cameralot::CameraFeed *feed);

extern "C" bool camera_feed_open_api_pref(
        cameralot::CameraFeed *cameraFeed,
        int32_t index,
        int32_t api
);

extern "C" bool camera_feed_open(cameralot::CameraFeed *cameraFeed, int32_t index);

extern "C" bool camera_feed_is_opened(cameralot::CameraFeed *cameraFeed);

extern "C" ReadStatus camera_feed_read(
        cameralot::CameraFeed *cameraFeed,
        uint32_t width,
        uint32_t height,
        char *ext,
        TimerData *td
);

extern "C" bool camera_feed_get_buf(
        cameralot::CameraFeed *cameraFeed,
        ByteBufferShare *buf
);