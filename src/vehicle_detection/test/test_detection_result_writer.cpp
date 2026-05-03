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

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "vehicle_detection/detection_json.hpp"
#include "vehicle_detection/detection_result_writer.hpp"

using vehicle_detection::DetectionJsonItem;
using vehicle_detection::DetectionJsonPayload;
using vehicle_detection::DetectionResultFormat;
using vehicle_detection::DetectionResultWriter;
using vehicle_detection::DetectionResultWriterConfig;
using vehicle_detection::parse_detection_result_format;

namespace
{

class TempDir
{
public:
  TempDir()
  {
    std::random_device rd;
    std::mt19937_64 gen{rd()};
    std::uniform_int_distribution<std::uint64_t> dist;
    std::ostringstream oss;
    oss << "vehicle_detection_result_writer_test_" << dist(gen);
    path_ = std::filesystem::temp_directory_path() / oss.str();
    std::filesystem::create_directories(path_);
  }

  ~TempDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;

  std::filesystem::path file(const std::string & name) const
  {
    return path_ / name;
  }

  const std::filesystem::path & path() const {return path_;}

private:
  std::filesystem::path path_;
};

DetectionJsonPayload make_payload(
  std::int32_t sec, std::uint32_t nanosec, const std::string & id,
  double cx, double cy)
{
  DetectionJsonPayload payload{};
  payload.stamp_sec = sec;
  payload.stamp_nanosec = nanosec;
  payload.frame_id = "map";

  DetectionJsonItem item;
  item.id = id;
  item.class_label = "car";
  item.confidence = 0.8;
  item.center_x = cx;
  item.center_y = cy;
  item.center_z = 0.0;
  item.length = 4.5;
  item.width = 1.8;
  item.height = 1.6;
  item.yaw = 0.0;
  payload.detections.push_back(item);
  return payload;
}

std::vector<std::string> read_lines(const std::filesystem::path & path)
{
  std::vector<std::string> lines;
  std::ifstream in(path, std::ios::binary);
  std::string line;
  while (std::getline(in, line)) {
    lines.push_back(line);
  }
  return lines;
}

}  // namespace

TEST(ParseDetectionResultFormat, AcceptsJsonl)
{
  DetectionResultFormat fmt = DetectionResultFormat::kJsonLines;
  const auto r = parse_detection_result_format("jsonl", &fmt);
  EXPECT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(fmt, DetectionResultFormat::kJsonLines);
}

TEST(ParseDetectionResultFormat, NullOutPointerStillSucceeds)
{
  const auto r = parse_detection_result_format("jsonl", nullptr);
  EXPECT_TRUE(r.ok) << r.reason;
}

TEST(ParseDetectionResultFormat, RejectsUnknown)
{
  DetectionResultFormat fmt = DetectionResultFormat::kJsonLines;
  const auto r = parse_detection_result_format("csv", &fmt);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("result_output_format"), std::string::npos);
  EXPECT_NE(r.reason.find("csv"), std::string::npos);
}

TEST(ParseDetectionResultFormat, RejectsCaseSensitive)
{
  const auto r = parse_detection_result_format("JSONL", nullptr);
  EXPECT_FALSE(r.ok);
}

TEST(DetectionResultWriter, DefaultIsDisabledAndAppendIsNoop)
{
  DetectionResultWriter writer;
  EXPECT_FALSE(writer.is_open());
  EXPECT_TRUE(writer.last_error().empty());

  const auto payload = make_payload(0, 0, "1", 1.0, 2.0);
  EXPECT_FALSE(writer.append(payload));
}

TEST(DetectionResultWriter, ConfigureDisabledSucceedsWithoutPath)
{
  DetectionResultWriter writer;

  DetectionResultWriterConfig cfg;
  cfg.enabled = false;
  cfg.path = "";
  cfg.format = DetectionResultFormat::kJsonLines;

  const auto r = writer.configure(cfg);
  EXPECT_TRUE(r.ok) << r.reason;
  EXPECT_FALSE(writer.is_open());
  EXPECT_TRUE(writer.last_error().empty());
}

TEST(DetectionResultWriter, EnabledRejectsEmptyPath)
{
  DetectionResultWriter writer;

  DetectionResultWriterConfig cfg;
  cfg.enabled = true;
  cfg.path = "";

  const auto r = writer.configure(cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("result_output_path"), std::string::npos);
  EXPECT_FALSE(writer.is_open());
  EXPECT_FALSE(writer.last_error().empty());
}

TEST(DetectionResultWriter, EnabledFailsWhenPathIsDirectory)
{
  TempDir tmp;

  DetectionResultWriter writer;
  DetectionResultWriterConfig cfg;
  cfg.enabled = true;
  // Asking ofstream to open the temp directory itself for write is
  // expected to fail. Verifies the writer downgrades to disabled
  // without throwing.
  cfg.path = tmp.path().string();

  const auto r = writer.configure(cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("failed to open"), std::string::npos);
  EXPECT_FALSE(writer.is_open());

  const auto payload = make_payload(0, 0, "1", 1.0, 2.0);
  EXPECT_FALSE(writer.append(payload));
}

TEST(DetectionResultWriter, EnabledFailsWhenParentMissing)
{
  TempDir tmp;
  const auto bad = tmp.path() / "no_such_subdir" / "out.jsonl";

  DetectionResultWriter writer;
  DetectionResultWriterConfig cfg;
  cfg.enabled = true;
  cfg.path = bad.string();

  const auto r = writer.configure(cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(writer.is_open());
  EXPECT_NE(writer.last_error().find("failed to open"), std::string::npos);
}

TEST(DetectionResultWriter, EnabledAppendsOneJsonlLinePerPayload)
{
  TempDir tmp;
  const auto path = tmp.file("results.jsonl");

  DetectionResultWriter writer;
  DetectionResultWriterConfig cfg;
  cfg.enabled = true;
  cfg.path = path.string();
  ASSERT_TRUE(writer.configure(cfg).ok);
  EXPECT_TRUE(writer.is_open());

  EXPECT_TRUE(writer.append(make_payload(1, 0, "1", 10.0, -2.0)));
  EXPECT_TRUE(writer.append(make_payload(2, 0, "2", 11.0, -3.0)));

  const auto lines = read_lines(path);
  ASSERT_EQ(lines.size(), 2u);

  // Each line is a single JSON object: starts with '{' and ends with '}'.
  for (const auto & line : lines) {
    ASSERT_FALSE(line.empty());
    EXPECT_EQ(line.front(), '{');
    EXPECT_EQ(line.back(), '}');
  }
  EXPECT_NE(lines[0].find("\"id\":\"1\""), std::string::npos);
  EXPECT_NE(lines[1].find("\"id\":\"2\""), std::string::npos);
  EXPECT_NE(lines[0].find("\"frame_id\":\"map\""), std::string::npos);
}

TEST(DetectionResultWriter, AppendsArePreservedAcrossReconfigure)
{
  TempDir tmp;
  const auto path = tmp.file("results.jsonl");

  DetectionResultWriter writer;
  DetectionResultWriterConfig cfg;
  cfg.enabled = true;
  cfg.path = path.string();
  ASSERT_TRUE(writer.configure(cfg).ok);
  ASSERT_TRUE(writer.append(make_payload(1, 0, "1", 1.0, 1.0)));

  // Disable and re-enable on the same path: append mode keeps the prior
  // line and appends the new one after it.
  DetectionResultWriterConfig disabled;
  disabled.enabled = false;
  ASSERT_TRUE(writer.configure(disabled).ok);
  EXPECT_FALSE(writer.is_open());

  ASSERT_TRUE(writer.configure(cfg).ok);
  ASSERT_TRUE(writer.append(make_payload(2, 0, "2", 2.0, 2.0)));

  const auto lines = read_lines(path);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_NE(lines[0].find("\"id\":\"1\""), std::string::npos);
  EXPECT_NE(lines[1].find("\"id\":\"2\""), std::string::npos);
}

TEST(DetectionResultWriter, FailedConfigureClosesPriorFile)
{
  TempDir tmp;
  const auto good = tmp.file("good.jsonl");

  DetectionResultWriter writer;
  DetectionResultWriterConfig cfg;
  cfg.enabled = true;
  cfg.path = good.string();
  ASSERT_TRUE(writer.configure(cfg).ok);
  ASSERT_TRUE(writer.is_open());

  // Invalid second configuration: enabled but empty path. The writer
  // must close the prior file and become disabled, without throwing.
  DetectionResultWriterConfig bad;
  bad.enabled = true;
  bad.path = "";
  const auto r = writer.configure(bad);
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(writer.is_open());

  // Subsequent appends are silent no-ops.
  EXPECT_FALSE(writer.append(make_payload(0, 0, "x", 0.0, 0.0)));
}

TEST(DetectionResultWriter, EmptyPayloadProducesValidLine)
{
  TempDir tmp;
  const auto path = tmp.file("empty.jsonl");

  DetectionResultWriter writer;
  DetectionResultWriterConfig cfg;
  cfg.enabled = true;
  cfg.path = path.string();
  ASSERT_TRUE(writer.configure(cfg).ok);

  DetectionJsonPayload empty{};
  empty.frame_id = "map";
  EXPECT_TRUE(writer.append(empty));

  const auto lines = read_lines(path);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_NE(lines[0].find("\"detections\":[]"), std::string::npos);
  EXPECT_NE(lines[0].find("\"frame_id\":\"map\""), std::string::npos);
}
