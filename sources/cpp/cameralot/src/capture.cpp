#include "capture.h"

using namespace cameralot;

OpenCVCameraFeed *opencv_camera_feed_create()
{
    return new OpenCVCameraFeed();
}

void opencv_camera_feed_delete(OpenCVCameraFeed *feed)
{
    delete feed;
}

bool opencv_camera_feed_open_api_pref(OpenCVCameraFeed *cameraFeed, int32_t index, int32_t api)
{
    return cameraFeed->open(index, api);
}

bool opencv_camera_feed_open(OpenCVCameraFeed *cameraFeed, int32_t index)
{
    return cameraFeed->open(index);
}

bool camera_feed_is_opened(CameraFeed *cameraFeed)
{
    return cameraFeed->is_opened();
}

ReadStatus abstract_camera_feed_read(
        AbstractCameraFeed *cameraFeed,
        uint32_t width,
        uint32_t height,
        const char *ext,
        TimerData *td,
        ByteBufferShare *buf
)
{
    return cameraFeed->read(width, height, ext, *td, buf);
}