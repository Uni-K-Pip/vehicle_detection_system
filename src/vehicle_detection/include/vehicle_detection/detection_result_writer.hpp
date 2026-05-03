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

#ifndef VEHICLE_DETECTION__DETECTION_RESULT_WRITER_HPP_
#define VEHICLE_DETECTION__DETECTION_RESULT_WRITER_HPP_

#include <fstream>
#include <mutex>
#include <string>

#include "vehicle_detection/detection_json.hpp"

namespace vehicle_detection
{

// Supported on-disk formats for the detection result writer.
enum class DetectionResultFormat
{
  kJsonLines,
};

struct DetectionResultWriterConfig
{
  // When false the writer becomes a no-op regardless of `path` / `format`.
  bool enabled{false};
  std::string path;
  DetectionResultFormat format{DetectionResultFormat::kJsonLines};
};

// Outcome of a configuration call. `ok=true` always implies the writer is
// in a stable state (either disabled or open for append). `ok=false`
// carries a human-readable reason in `reason` and leaves the writer
// disabled.
struct DetectionResultWriterResult
{
  bool ok{true};
  std::string reason;

  static DetectionResultWriterResult success();
  static DetectionResultWriterResult failure(std::string reason);
};

// Parse a textual format name into the enum. Currently accepts only
// "jsonl" (case-sensitive). On success, writes the parsed value into
// `*out` when `out` is non-null.
DetectionResultWriterResult parse_detection_result_format(
  const std::string & name, DetectionResultFormat * out);

// Append-only sink for serialized detection payloads. Currently writes
// JSON Lines (`.jsonl`): one detection payload per line, in the same
// schema as the HTTP POST body emitted by `serialize_detections`.
//
// All public methods are thread-safe. Configuration errors and write
// failures never throw; they leave the writer disabled and surface the
// reason via `last_error()`. Callers (e.g. `detection_sender_node`) are
// expected to log the reason and continue running.
class DetectionResultWriter
{
public:
  DetectionResultWriter();
  ~DetectionResultWriter();

  DetectionResultWriter(const DetectionResultWriter &) = delete;
  DetectionResultWriter & operator=(const DetectionResultWriter &) = delete;

  // Apply a new configuration. Closes any prior open file before
  // re-opening. Returns success when the configuration disables saving
  // (regardless of `path` / `format`), or when enabled and the file was
  // opened for append. Returns failure when the configuration is invalid
  // (empty path while enabled) or when the file could not be opened.
  // After a failure the writer is left disabled so subsequent appends
  // are no-ops.
  DetectionResultWriterResult configure(
    const DetectionResultWriterConfig & config);

  // Append one detection payload as a single JSONL line. Adds a trailing
  // '\n' to keep the file parseable line-by-line. Returns true when the
  // line was successfully written, false when the writer is disabled,
  // not open, or the underlying write failed (in which case the writer
  // becomes disabled and `last_error()` is updated).
  bool append(const DetectionJsonPayload & payload);

  // True when the writer is currently enabled and the underlying file
  // is open for append.
  bool is_open() const;

  // Most recent error reason. Empty when no error has occurred since the
  // last successful configure / append.
  std::string last_error() const;

private:
  mutable std::mutex mutex_;
  bool enabled_;
  DetectionResultFormat format_;
  std::string path_;
  std::ofstream out_;
  std::string last_error_;
};

}  // namespace vehicle_detection

#endif  // VEHICLE_DETECTION__DETECTION_RESULT_WRITER_HPP_
