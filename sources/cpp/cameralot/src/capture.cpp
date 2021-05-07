#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "capture.h"

using namespace cameralot;

ReadStatus CameraFeed::read(uint32_t width, uint32_t height, const char *ext) noexcept
{
    this->image_buffer.clear();
    this->has_image = false;

    ReadStatus status = ReadStatus::Success;

    if (!status && !this->vision.get_frame(*(this->reading_frame))) {
        status = ReadStatus::RetrieveFailed;
    }

    if (!status && this->reading_frame.empty()) {
        status = ReadStatus::EmptyFrame;
    }

    if (!status) {
        auto s = cv::Size(width, height);

        cv::resize(this->reading_frame, this->output_frame, s);

        if (!cv::imencode(cv::String(ext), this->output_frame, this->image_buffer)) {
            status = ReadStatus::EncodingFailed;
        } else {
            this->has_image = true;
        }
    }

    return status;
}

bool CameraFeed::get_buffer(ByteBufferShare *buffer) noexcept
{
    if (this->has_image) {
        buffer->buffer = &this->image_buffer[0];
        buffer->length = this->image_buffer.size();
    }

    return this->has_image;
}