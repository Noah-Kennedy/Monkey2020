#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "capture.h"
#include "camera_feed.h"

using namespace cameralot;

bool OpenCVCameraFeed::is_opened() const noexcept
{
    return this->videoCapture.isOpened();
}

bool OpenCVCameraFeed::open(int index, int api_preference) noexcept
{
    return this->videoCapture.open(index, api_preference);
}

ReadStatus OpenCVCameraFeed::read_frame(cv::Mat &frame, TimerData &td) noexcept
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    ReadStatus status = ReadStatus::Success;

    // start timer for grab
    auto t1 = high_resolution_clock::now();

    if (!this->videoCapture.grab()) {
        status = ReadStatus::ReadFailed;
    }

    // record grab time
    auto t2 = high_resolution_clock::now();
    td.grab_millis = duration_cast<milliseconds>(t2 - t1).count();

    // start timer for retrieve
    t1 = high_resolution_clock::now();
    if (status == ReadStatus::Success && !this->videoCapture.retrieve(frame)) {
        status = ReadStatus::RetrieveFailed;
    }

    // record retrieve time
    t2 = high_resolution_clock::now();
    td.retrieve_millis = duration_cast<milliseconds>(t2 - t1).count();

    return status;
}

ReadStatus CameraFeed::read(
        uint32_t width,
        uint32_t height,
        const char *ext,
        TimerData &td,
        ByteBufferShare *buffer
) noexcept
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    this->image_buffer.clear();

    ReadStatus status = ReadStatus::Success;

    if (!this->is_opened()) {
        status = ReadStatus::NotOpen;
    }

    if (status == ReadStatus::Success) {
        status = this->read_frame(this->reading_frame, td);
    }

    if (status == ReadStatus::Success && this->reading_frame.empty()) {
        status = ReadStatus::EmptyFrame;
    }

    if (status == ReadStatus::Success) {
        auto s = cv::Size(width, height);

        auto t1 = high_resolution_clock::now();
        cv::resize(this->reading_frame, this->output_frame, s);
        auto t2 = high_resolution_clock::now();
        td.resize_millis = duration_cast<milliseconds>(t2 - t1).count();

        t1 = high_resolution_clock::now();
        if (cv::imencode(cv::String(ext), this->output_frame, this->image_buffer)) {
            buffer->buffer = &this->image_buffer[0];
            buffer->length = this->image_buffer.size();
        } else {
            status = ReadStatus::EncodingFailed;
        }

        t2 = high_resolution_clock::now();
        td.encode_millis = duration_cast<milliseconds>(t2 - t1).count();
    }

    return status;
}