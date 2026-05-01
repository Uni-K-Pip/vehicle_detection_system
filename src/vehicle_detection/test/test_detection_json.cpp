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

#include <gtest/gtest.h>

#include <string>

#include "vehicle_detection/detection_json.hpp"
#include "vehicle_detection/http_client.hpp"

using vehicle_detection::DetectionJsonItem;
using vehicle_detection::DetectionJsonPayload;
using vehicle_detection::HttpUrl;
using vehicle_detection::format_iso8601_utc;
using vehicle_detection::parse_http_url;
using vehicle_detection::serialize_detections;

TEST(DetectionJson, FormatIso8601UtcEpoch)
{
  EXPECT_EQ(format_iso8601_utc(0, 0), "1970-01-01T00:00:00.000Z");
}

TEST(DetectionJson, FormatIso8601UtcNanosToMillis)
{
  // 1 second past epoch, 123,456,789 ns -> 123 ms.
  EXPECT_EQ(format_iso8601_utc(1, 123456789U), "1970-01-01T00:00:01.123Z");
}

TEST(DetectionJson, EmptyPayload)
{
  DetectionJsonPayload payload{};
  payload.stamp_sec = 0;
  payload.stamp_nanosec = 0;
  payload.frame_id = "map";
  const std::string s = serialize_detections(payload);
  EXPECT_NE(s.find("\"frame_id\":\"map\""), std::string::npos);
  EXPECT_NE(s.find("\"detections\":[]"), std::string::npos);
  EXPECT_NE(s.find("\"timestamp\""), std::string::npos);
}

TEST(DetectionJson, SingleDetection)
{
  DetectionJsonPayload payload{};
  payload.stamp_sec = 0;
  payload.stamp_nanosec = 0;
  payload.frame_id = "map";
  DetectionJsonItem item;
  item.id = "1";
  item.class_label = "car";
  item.confidence = 0.8;
  item.center_x = 1.5;
  item.center_y = -2.5;
  item.center_z = 0.0;
  item.length = 4.5;
  item.width = 1.8;
  item.height = 1.6;
  item.yaw = 0.0;
  payload.detections.push_back(item);

  const std::string s = serialize_detections(payload);
  EXPECT_NE(s.find("\"id\":\"1\""), std::string::npos);
  EXPECT_NE(s.find("\"class\":\"car\""), std::string::npos);
  EXPECT_NE(s.find("\"length\":4.500000"), std::string::npos);
  EXPECT_NE(s.find("\"width\":1.800000"), std::string::npos);
  EXPECT_NE(s.find("\"height\":1.600000"), std::string::npos);
  EXPECT_NE(s.find("\"x\":1.500000"), std::string::npos);
  EXPECT_NE(s.find("\"y\":-2.500000"), std::string::npos);
}

TEST(DetectionJson, EscapesQuotesAndBackslashes)
{
  DetectionJsonPayload payload{};
  payload.frame_id = "weird\"frame\\";
  const std::string s = serialize_detections(payload);
  EXPECT_NE(s.find("\"frame_id\":\"weird\\\"frame\\\\\""), std::string::npos);
}

TEST(HttpClient, ParsesHttpUrlWithPort)
{
  HttpUrl out;
  ASSERT_TRUE(parse_http_url("http://host.docker.internal:8080/detections", &out));
  EXPECT_EQ(out.host, "host.docker.internal");
  EXPECT_EQ(out.port, "8080");
  EXPECT_EQ(out.path, "/detections");
}

TEST(HttpClient, ParsesHttpUrlDefaultPort)
{
  HttpUrl out;
  ASSERT_TRUE(parse_http_url("http://example.com/api", &out));
  EXPECT_EQ(out.host, "example.com");
  EXPECT_EQ(out.port, "80");
  EXPECT_EQ(out.path, "/api");
}

TEST(HttpClient, ParsesHttpUrlNoPath)
{
  HttpUrl out;
  ASSERT_TRUE(parse_http_url("http://example.com:9000", &out));
  EXPECT_EQ(out.host, "example.com");
  EXPECT_EQ(out.port, "9000");
  EXPECT_EQ(out.path, "/");
}

TEST(HttpClient, RejectsHttps)
{
  HttpUrl out;
  EXPECT_FALSE(parse_http_url("https://example.com/api", &out));
}

TEST(HttpClient, RejectsEmptyAuthority)
{
  HttpUrl out;
  EXPECT_FALSE(parse_http_url("http:///api", &out));
}
