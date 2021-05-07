/*************************************************************************************************
 * Author: Noah Kennedy
 *
 * Description:
 *      This file provides a set of C bindings for the CameraFeed class.
 *      This allows Rust (and other languages) to bind to Cameralot.
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

#include <opencv2/videoio.hpp>
#include "camera_feed.h"

using namespace cameralot;

extern "C" OpenCVCameraFeed *opencv_camera_feed_create();
extern "C" void opencv_camera_feed_delete(cameralot::OpenCVCameraFeed *feed);

extern "C" bool opencv_camera_feed_open_api_pref(
        OpenCVCameraFeed *cameraFeed,
        int32_t index,
        int32_t api
);

extern "C" bool opencv_camera_feed_open(OpenCVCameraFeed *cameraFeed, int32_t index);

extern "C" bool camera_feed_is_opened(CameraFeed *cameraFeed);

extern "C" ReadStatus abstract_camera_feed_read(
        AbstractCameraFeed *cameraFeed,
        uint32_t width,
        uint32_t height,
        const char *ext,
        TimerData *td,
        ByteBufferShare *buf
);