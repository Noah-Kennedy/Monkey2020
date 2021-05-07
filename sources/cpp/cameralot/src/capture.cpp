#include "capture.h"

using namespace cameralot;

CameraFeed *camera_feed_create()
{
    return new CameraFeed();
}

void camera_feed_delete(CameraFeed *feed)
{
    delete feed;
}

bool camera_feed_open_api_pref(CameraFeed *cameraFeed, int32_t index, int32_t api)
{
    return cameraFeed->open(index, api);
}

bool camera_feed_open(CameraFeed *cameraFeed, int32_t index)
{
    return cameraFeed->open(index);
}

bool camera_feed_is_opened(CameraFeed *cameraFeed)
{
    return cameraFeed->is_opened();
}

ReadStatus camera_feed_read(CameraFeed *cameraFeed, uint32_t width, uint32_t height, char *ext, TimerData* td)
{
    return cameraFeed->read(width, height, ext, td);
}

bool camera_feed_get_buf(CameraFeed *cameraFeed, ByteBufferShare *buf)
{
    return cameraFeed->get_buffer(buf);
}