/*************************************************************************************************
 * Author: Noah Kennedy
 *
 * Description:
 *      Cameralot is a small library that just wraps opencv to make it easier to serve camera feeds.
 *      It was written to be easy to bind to other languages.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Lesser Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************************************/

#pragma once

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