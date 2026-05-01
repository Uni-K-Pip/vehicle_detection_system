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

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <utility>

#include "vehicle_detection/parameter_validation.hpp"

namespace vehicle_detection
{

ValidationResult ValidationResult::success()
{
  return ValidationResult{true, std::string{}};
}

ValidationResult ValidationResult::failure(std::string reason)
{
  return ValidationResult{false, std::move(reason)};
}

ValidationResult validate_non_empty_string(
  std::string_view name,
  std::string_view value)
{
  if (value.empty()) {
    std::ostringstream oss;
    oss << "parameter '" << name << "' must not be empty";
    return ValidationResult::failure(oss.str());
  }
  return ValidationResult::success();
}

ValidationResult validate_existing_file(
  std::string_view name,
  std::string_view path)
{
  if (auto r = validate_non_empty_string(name, path); !r.ok) {
    return r;
  }
  std::error_code ec;
  const std::filesystem::path fs_path{std::string{path}};
  const bool exists = std::filesystem::exists(fs_path, ec);
  if (ec || !exists) {
    std::ostringstream oss;
    oss << "parameter '" << name << "' references missing file: " << path;
    return ValidationResult::failure(oss.str());
  }
  if (std::filesystem::is_directory(fs_path, ec)) {
    std::ostringstream oss;
    oss << "parameter '" << name << "' must be a file, not a directory: " << path;
    return ValidationResult::failure(oss.str());
  }
  return ValidationResult::success();
}

ValidationResult validate_positive_double(
  std::string_view name,
  double value)
{
  if (!(value > 0.0)) {
    std::ostringstream oss;
    oss << "parameter '" << name << "' must be > 0.0 (got " << value << ")";
    return ValidationResult::failure(oss.str());
  }
  return ValidationResult::success();
}

ValidationResult validate_double_range(
  std::string_view name,
  double value,
  double min_inclusive,
  double max_inclusive)
{
  if (value < min_inclusive || value > max_inclusive) {
    std::ostringstream oss;
    oss << "parameter '" << name << "' must be in ["
        << min_inclusive << ", " << max_inclusive << "] (got " << value << ")";
    return ValidationResult::failure(oss.str());
  }
  return ValidationResult::success();
}

ValidationResult validate_int_min(
  std::string_view name,
  std::int64_t value,
  std::int64_t min_inclusive)
{
  if (value < min_inclusive) {
    std::ostringstream oss;
    oss << "parameter '" << name << "' must be >= " << min_inclusive
        << " (got " << value << ")";
    return ValidationResult::failure(oss.str());
  }
  return ValidationResult::success();
}

ValidationResult validate_enum(
  std::string_view name,
  std::string_view value,
  std::initializer_list<std::string_view> allowed)
{
  const auto found = std::find(allowed.begin(), allowed.end(), value);
  if (found == allowed.end()) {
    std::ostringstream oss;
    oss << "parameter '" << name << "' must be one of {";
    bool first = true;
    for (const auto & a : allowed) {
      if (!first) {
        oss << ", ";
      }
      oss << a;
      first = false;
    }
    oss << "} (got '" << value << "')";
    return ValidationResult::failure(oss.str());
  }
  return ValidationResult::success();
}

}  // namespace vehicle_detection
