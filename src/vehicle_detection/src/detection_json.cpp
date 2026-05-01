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

#include "vehicle_detection/detection_json.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <string>

namespace vehicle_detection
{

namespace
{

std::string format_double(double value)
{
  if (!std::isfinite(value)) {
    return "0.0";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.6f", value);
  return std::string(buffer);
}

std::string escape_json_string(const std::string & input)
{
  std::string out;
  out.reserve(input.size() + 2);
  for (const char c : input) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x",
            static_cast<unsigned int>(c));
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }
  return out;
}

}  // namespace

std::string format_iso8601_utc(std::int32_t sec, std::uint32_t nanosec)
{
  time_t t = static_cast<time_t>(sec);
  struct tm tm_utc;
#if defined(_WIN32)
  gmtime_s(&tm_utc, &t);
#else
  gmtime_r(&t, &tm_utc);
#endif
  char base[32];
  std::strftime(base, sizeof(base), "%Y-%m-%dT%H:%M:%S", &tm_utc);
  char out[48];
  const std::uint32_t millis = nanosec / 1000000U;
  std::snprintf(out, sizeof(out), "%s.%03uZ", base, millis);
  return std::string(out);
}

std::string serialize_detections(const DetectionJsonPayload & payload)
{
  std::ostringstream os;
  os << "{";
  os << "\"timestamp\":\""
     << format_iso8601_utc(payload.stamp_sec, payload.stamp_nanosec) << "\"";
  os << ",\"frame_id\":\"" << escape_json_string(payload.frame_id) << "\"";
  os << ",\"detections\":[";
  for (std::size_t i = 0; i < payload.detections.size(); ++i) {
    if (i > 0) {
      os << ",";
    }
    const auto & d = payload.detections[i];
    os << "{";
    os << "\"id\":\"" << escape_json_string(d.id) << "\"";
    os << ",\"class\":\"" << escape_json_string(d.class_label) << "\"";
    os << ",\"confidence\":" << format_double(d.confidence);
    os << ",\"center\":{"
       << "\"x\":" << format_double(d.center_x)
       << ",\"y\":" << format_double(d.center_y)
       << ",\"z\":" << format_double(d.center_z)
       << "}";
    os << ",\"size\":{"
       << "\"length\":" << format_double(d.length)
       << ",\"width\":" << format_double(d.width)
       << ",\"height\":" << format_double(d.height)
       << "}";
    os << ",\"yaw\":" << format_double(d.yaw);
    os << "}";
  }
  os << "]}";
  return os.str();
}

}  // namespace vehicle_detection
