#ifndef VEHICLE_DETECTION__PARAMETER_VALIDATION_HPP_
#define VEHICLE_DETECTION__PARAMETER_VALIDATION_HPP_

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace vehicle_detection
{

struct ValidationResult
{
  bool ok;
  std::string reason;

  static ValidationResult success();
  static ValidationResult failure(std::string reason);
};

ValidationResult validate_non_empty_string(
  std::string_view name,
  std::string_view value);

ValidationResult validate_existing_file(
  std::string_view name,
  std::string_view path);

ValidationResult validate_positive_double(
  std::string_view name,
  double value);

ValidationResult validate_double_range(
  std::string_view name,
  double value,
  double min_inclusive,
  double max_inclusive);

ValidationResult validate_int_min(
  std::string_view name,
  long long value,
  long long min_inclusive);

ValidationResult validate_enum(
  std::string_view name,
  std::string_view value,
  std::initializer_list<std::string_view> allowed);

}  // namespace vehicle_detection

#endif  // VEHICLE_DETECTION__PARAMETER_VALIDATION_HPP_
