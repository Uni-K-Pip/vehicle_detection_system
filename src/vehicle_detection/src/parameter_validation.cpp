#include "vehicle_detection/parameter_validation.hpp"

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <utility>

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
  long long value,
  long long min_inclusive)
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
