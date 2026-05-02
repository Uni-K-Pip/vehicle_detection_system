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

#include "vehicle_detection/parameter_json.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/parameter_value.hpp>

namespace vehicle_detection
{

using nlohmann::json;
using rclcpp::ParameterType;

namespace
{

[[noreturn]] void throw_conversion(
  const json & value,
  const std::string & target_type)
{
  std::ostringstream oss;
  oss << "cannot convert JSON value '" << value.dump()
      << "' to parameter type '" << target_type << "'";
  throw std::invalid_argument(oss.str());
}

double json_to_double(const json & value)
{
  if (value.is_number()) {
    return value.get<double>();
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? 1.0 : 0.0;
  }
  throw_conversion(value, "double");
}

int64_t json_to_int(const json & value)
{
  // is_number_unsigned() must be checked before is_number_integer():
  // nlohmann/json reports unsigned integers as true for both, and
  // get<int64_t>() on an unsigned value above INT64_MAX silently wraps.
  if (value.is_number_unsigned()) {
    const auto u = value.get<uint64_t>();
    if (u > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
      throw_conversion(value, "integer");
    }
    return static_cast<int64_t>(u);
  }
  if (value.is_number_integer()) {
    return value.get<int64_t>();
  }
  if (value.is_number_float()) {
    const double d = value.get<double>();
    if (!std::isfinite(d) || std::trunc(d) != d) {
      throw_conversion(value, "integer");
    }
    // int64_t is exactly -2^63 .. 2^63 - 1. Both bounds are representable
    // as double, but static_cast<double>(INT64_MAX) rounds up to 2^63, so
    // compare against the exclusive upper bound 2^63 (= -INT64_MIN as double)
    // to keep the subsequent cast in range.
    constexpr double min_d =
      static_cast<double>(std::numeric_limits<int64_t>::min());
    constexpr double upper_exclusive = -min_d;
    if (d < min_d || d >= upper_exclusive) {
      throw_conversion(value, "integer");
    }
    return static_cast<int64_t>(d);
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? 1 : 0;
  }
  throw_conversion(value, "integer");
}

bool json_to_bool(const json & value)
{
  if (value.is_boolean()) {
    return value.get<bool>();
  }
  if (value.is_number()) {
    return value.get<double>() != 0.0;
  }
  if (value.is_string()) {
    const auto & s = value.get_ref<const std::string &>();
    if (s == "true" || s == "1") {return true;}
    if (s == "false" || s == "0") {return false;}
  }
  throw_conversion(value, "bool");
}

std::string json_to_string(const json & value)
{
  if (value.is_string()) {
    return value.get<std::string>();
  }
  if (value.is_number() || value.is_boolean()) {
    return value.dump();
  }
  throw_conversion(value, "string");
}

}  // namespace

std::string parameter_type_name(std::uint8_t type)
{
  switch (static_cast<ParameterType>(type)) {
    case ParameterType::PARAMETER_NOT_SET: return "not_set";
    case ParameterType::PARAMETER_BOOL: return "bool";
    case ParameterType::PARAMETER_INTEGER: return "integer";
    case ParameterType::PARAMETER_DOUBLE: return "double";
    case ParameterType::PARAMETER_STRING: return "string";
    case ParameterType::PARAMETER_BYTE_ARRAY: return "byte_array";
    case ParameterType::PARAMETER_BOOL_ARRAY: return "bool_array";
    case ParameterType::PARAMETER_INTEGER_ARRAY: return "integer_array";
    case ParameterType::PARAMETER_DOUBLE_ARRAY: return "double_array";
    case ParameterType::PARAMETER_STRING_ARRAY: return "string_array";
  }
  return "unknown";
}

nlohmann::json parameter_to_json(const rclcpp::Parameter & param)
{
  json out;
  out["name"] = param.get_name();
  const auto type = static_cast<std::uint8_t>(param.get_type());
  out["type"] = parameter_type_name(type);

  switch (param.get_type()) {
    case ParameterType::PARAMETER_NOT_SET:
      out["value"] = nullptr;
      break;
    case ParameterType::PARAMETER_BOOL:
      out["value"] = param.as_bool();
      break;
    case ParameterType::PARAMETER_INTEGER:
      out["value"] = param.as_int();
      break;
    case ParameterType::PARAMETER_DOUBLE:
      out["value"] = param.as_double();
      break;
    case ParameterType::PARAMETER_STRING:
      out["value"] = param.as_string();
      break;
    case ParameterType::PARAMETER_BYTE_ARRAY:
      out["value"] = param.as_byte_array();
      break;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      out["value"] = param.as_bool_array();
      break;
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      out["value"] = param.as_integer_array();
      break;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      out["value"] = param.as_double_array();
      break;
    case ParameterType::PARAMETER_STRING_ARRAY:
      out["value"] = param.as_string_array();
      break;
  }
  return out;
}

rclcpp::ParameterValue json_to_parameter_value(
  const json & value,
  std::uint8_t expected_type)
{
  const auto type = static_cast<ParameterType>(expected_type);
  switch (type) {
    case ParameterType::PARAMETER_BOOL:
      return rclcpp::ParameterValue(json_to_bool(value));
    case ParameterType::PARAMETER_INTEGER:
      return rclcpp::ParameterValue(static_cast<int64_t>(json_to_int(value)));
    case ParameterType::PARAMETER_DOUBLE:
      return rclcpp::ParameterValue(json_to_double(value));
    case ParameterType::PARAMETER_STRING:
      return rclcpp::ParameterValue(json_to_string(value));
    case ParameterType::PARAMETER_BOOL_ARRAY: {
        if (!value.is_array()) {throw_conversion(value, "bool_array");}
        std::vector<bool> arr;
        arr.reserve(value.size());
        for (const auto & v : value) {
          arr.push_back(json_to_bool(v));
        }
        return rclcpp::ParameterValue(arr);
      }
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
        if (!value.is_array()) {throw_conversion(value, "integer_array");}
        std::vector<int64_t> arr;
        arr.reserve(value.size());
        for (const auto & v : value) {
          arr.push_back(static_cast<int64_t>(json_to_int(v)));
        }
        return rclcpp::ParameterValue(arr);
      }
    case ParameterType::PARAMETER_DOUBLE_ARRAY: {
        if (!value.is_array()) {throw_conversion(value, "double_array");}
        std::vector<double> arr;
        arr.reserve(value.size());
        for (const auto & v : value) {
          arr.push_back(json_to_double(v));
        }
        return rclcpp::ParameterValue(arr);
      }
    case ParameterType::PARAMETER_STRING_ARRAY: {
        if (!value.is_array()) {throw_conversion(value, "string_array");}
        std::vector<std::string> arr;
        arr.reserve(value.size());
        for (const auto & v : value) {
          arr.push_back(json_to_string(v));
        }
        return rclcpp::ParameterValue(arr);
      }
    case ParameterType::PARAMETER_BYTE_ARRAY: {
        throw std::invalid_argument(
        "byte_array parameters are not supported via the GUI bridge");
      }
    case ParameterType::PARAMETER_NOT_SET:
      throw std::invalid_argument(
        "target parameter is not declared on the remote node");
  }
  throw std::invalid_argument("unknown parameter type");
}

}  // namespace vehicle_detection
