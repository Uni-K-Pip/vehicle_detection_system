// Copyright 2026 kohei
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef VEHICLE_DETECTION__DETECTION_JSON_HPP_
#define VEHICLE_DETECTION__DETECTION_JSON_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace vehicle_detection
{

struct DetectionJsonItem
{
  std::string id;
  std::string class_label;
  double confidence;
  double center_x;
  double center_y;
  double center_z;
  double length;
  double width;
  double height;
  double yaw;
};

struct DetectionJsonPayload
{
  std::int32_t stamp_sec;
  std::uint32_t stamp_nanosec;
  std::string frame_id;
  std::vector<DetectionJsonItem> detections;
};

std::string format_iso8601_utc(std::int32_t sec, std::uint32_t nanosec);

std::string serialize_detections(const DetectionJsonPayload & payload);

}  // namespace vehicle_detection

#endif  // VEHICLE_DETECTION__DETECTION_JSON_HPP_
