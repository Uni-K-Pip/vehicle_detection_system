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

#include "vehicle_detection/detection_result_writer.hpp"

#include <ios>
#include <utility>

namespace vehicle_detection
{

DetectionResultWriterResult DetectionResultWriterResult::success()
{
  return DetectionResultWriterResult{true, std::string{}};
}

DetectionResultWriterResult DetectionResultWriterResult::failure(
  std::string reason)
{
  return DetectionResultWriterResult{false, std::move(reason)};
}

DetectionResultWriterResult parse_detection_result_format(
  const std::string & name, DetectionResultFormat * out)
{
  if (name == "jsonl") {
    if (out != nullptr) {
      *out = DetectionResultFormat::kJsonLines;
    }
    return DetectionResultWriterResult::success();
  }
  return DetectionResultWriterResult::failure(
    "result_output_format must be 'jsonl' (got '" + name + "')");
}

DetectionResultWriter::DetectionResultWriter()
: enabled_(false), format_(DetectionResultFormat::kJsonLines)
{}

DetectionResultWriter::~DetectionResultWriter() = default;

DetectionResultWriterResult DetectionResultWriter::configure(
  const DetectionResultWriterConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Always close any prior file. Re-open below only when enabling
  // succeeds, so a failed reconfigure leaves the writer disabled.
  if (out_.is_open()) {
    out_.close();
  }
  out_.clear();
  enabled_ = false;
  path_.clear();
  format_ = DetectionResultFormat::kJsonLines;
  last_error_.clear();

  if (!config.enabled) {
    return DetectionResultWriterResult::success();
  }
  if (config.path.empty()) {
    last_error_ =
      "result_output_path must not be empty when save_results=true";
    return DetectionResultWriterResult::failure(last_error_);
  }

  // Append + binary so newlines stay '\n' on every platform and previous
  // runs at the same path are preserved.
  out_.open(
    config.path,
    std::ios::out | std::ios::app | std::ios::binary);
  if (!out_.is_open()) {
    last_error_ =
      "failed to open result_output_path for append: " + config.path;
    return DetectionResultWriterResult::failure(last_error_);
  }

  enabled_ = true;
  format_ = config.format;
  path_ = config.path;
  return DetectionResultWriterResult::success();
}

bool DetectionResultWriter::append(const DetectionJsonPayload & payload)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || !out_.is_open()) {
    return false;
  }
  // Only JSONL is supported today; other formats are rejected at
  // configure time.
  const std::string line = serialize_detections(payload);
  out_ << line << '\n';
  if (!out_) {
    last_error_ = "write to result_output_path failed: " + path_;
    out_.close();
    out_.clear();
    enabled_ = false;
    return false;
  }
  out_.flush();
  if (!out_) {
    last_error_ = "flush to result_output_path failed: " + path_;
    out_.close();
    out_.clear();
    enabled_ = false;
    return false;
  }
  return true;
}

bool DetectionResultWriter::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_ && out_.is_open();
}

std::string DetectionResultWriter::last_error() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

}  // namespace vehicle_detection
