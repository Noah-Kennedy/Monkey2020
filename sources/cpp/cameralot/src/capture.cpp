#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "capture.h"

using namespace cameralot;

bool CameraFeed::is_opened() const noexcept
{
    return this->videoCapture.isOpened();
}

bool CameraFeed::open(int index, int api_preference) noexcept
{
    return this->videoCapture.open(index, api_preference);
}

ReadStatus CameraFeed::read(uint32_t width, uint32_t height, const char *ext, TimerData *td) noexcept
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    this->image_buffer.clear();
    this->has_image = false;

    ReadStatus status = ReadStatus::Success;

    if (!this->is_opened()) {
        status = ReadStatus::NotOpen;
    }

    auto t1 = high_resolution_clock::now();
    if (!status && !this->videoCapture.grab()) {
        status = ReadStatus::ReadFailed;
    }
    auto t2 = high_resolution_clock::now();
    td->grab_millis = duration_cast<milliseconds>(t2 - t1).count();

    t1 = high_resolution_clock::now();
    if (!status && !this->videoCapture.retrieve(this->reading_frame)) {
        status = ReadStatus::RetrieveFailed;
    }
    t2 = high_resolution_clock::now();
    td->retrieve_millis = duration_cast<milliseconds>(t2 - t1).count();

    if (!status && this->reading_frame.empty()) {
        status = ReadStatus::EmptyFrame;
    }

    if (!status) {
        auto s = cv::Size(width, height);

        t1 = high_resolution_clock::now();
        cv::resize(this->reading_frame, this->output_frame, s);
        t2 = high_resolution_clock::now();
        td->resize_millis = duration_cast<milliseconds>(t2 - t1).count();

        t1 = high_resolution_clock::now();
        if (!cv::imencode(cv::String(ext), this->output_frame, this->image_buffer)) {
            status = ReadStatus::EncodingFailed;
        } else {
            this->has_image = true;
        }

        t2 = high_resolution_clock::now();
        td->encode_millis = duration_cast<milliseconds>(t2 - t1).count();
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