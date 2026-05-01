#ifndef VEHICLE_DETECTION__PARAMETER_JSON_HPP_
#define VEHICLE_DETECTION__PARAMETER_JSON_HPP_

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

namespace vehicle_detection
{

// Convert a rclcpp::Parameter to a JSON object of the form
// {"name": "...", "type": "...", "value": <native JSON value>}.
nlohmann::json parameter_to_json(const rclcpp::Parameter & param);

// Convert a raw JSON value into a rclcpp::ParameterValue, coercing to the
// expected ROS 2 parameter type (rclcpp::ParameterType cast to uint8_t).
// Throws std::invalid_argument if the JSON value cannot be represented as
// the requested type.
rclcpp::ParameterValue json_to_parameter_value(
  const nlohmann::json & value,
  std::uint8_t expected_type);

// Human-readable name for a parameter type, e.g. "double", "string".
std::string parameter_type_name(std::uint8_t type);

}  // namespace vehicle_detection

#endif  // VEHICLE_DETECTION__PARAMETER_JSON_HPP_
