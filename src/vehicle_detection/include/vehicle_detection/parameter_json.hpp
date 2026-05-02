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
