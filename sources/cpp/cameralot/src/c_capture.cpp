#include "capture.h"

using namespace cameralot;

CameraFeed *camera_feed_create(visual_processing::MonkeyVision *vision)
{
    return new CameraFeed(vision);
}

void camera_feed_delete(CameraFeed *feed)
{
    delete feed;
}

ReadStatus camera_feed_read(CameraFeed *cameraFeed, uint32_t width, uint32_t height, char *ext)
{
    return cameraFeed->read(width, height, ext);
}

bool camera_feed_get_buf(CameraFeed *cameraFeed, ByteBufferShare *buf)
{
    return cameraFeed->get_buffer(buf);
}