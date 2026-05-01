#include "vehicle_detection/parameter_json.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

using nlohmann::json;
using rclcpp::Parameter;
using rclcpp::ParameterType;
using rclcpp::ParameterValue;
using vehicle_detection::json_to_parameter_value;
using vehicle_detection::parameter_to_json;
using vehicle_detection::parameter_type_name;

TEST(ParameterTypeName, KnownTypesAreNamed)
{
  EXPECT_EQ(parameter_type_name(
    static_cast<std::uint8_t>(ParameterType::PARAMETER_BOOL)), "bool");
  EXPECT_EQ(parameter_type_name(
    static_cast<std::uint8_t>(ParameterType::PARAMETER_INTEGER)), "integer");
  EXPECT_EQ(parameter_type_name(
    static_cast<std::uint8_t>(ParameterType::PARAMETER_DOUBLE)), "double");
  EXPECT_EQ(parameter_type_name(
    static_cast<std::uint8_t>(ParameterType::PARAMETER_STRING)), "string");
  EXPECT_EQ(parameter_type_name(
    static_cast<std::uint8_t>(ParameterType::PARAMETER_DOUBLE_ARRAY)),
    "double_array");
}

TEST(ParameterToJson, EncodesScalars)
{
  EXPECT_EQ(parameter_to_json(Parameter("flag", true))["value"], true);
  EXPECT_EQ(parameter_to_json(Parameter("count", 7))["type"], "integer");
  EXPECT_EQ(parameter_to_json(Parameter("count", 7))["value"], 7);
  EXPECT_EQ(parameter_to_json(Parameter("ratio", 0.25))["type"], "double");
  EXPECT_DOUBLE_EQ(
    parameter_to_json(Parameter("ratio", 0.25))["value"].get<double>(),
    0.25);
  EXPECT_EQ(parameter_to_json(Parameter("name", std::string{"car"}))["value"],
    "car");
}

TEST(ParameterToJson, EncodesArrays)
{
  std::vector<double> doubles{1.0, 2.5, -3.0};
  auto j = parameter_to_json(Parameter("limits", doubles));
  EXPECT_EQ(j["type"], "double_array");
  ASSERT_TRUE(j["value"].is_array());
  ASSERT_EQ(j["value"].size(), 3u);
  EXPECT_DOUBLE_EQ(j["value"][1].get<double>(), 2.5);

  std::vector<std::string> strings{"a", "b"};
  auto js = parameter_to_json(Parameter("names", strings));
  EXPECT_EQ(js["type"], "string_array");
  EXPECT_EQ(js["value"][0], "a");
}

TEST(JsonToParameterValue, CoercesToExpectedTypes)
{
  const auto bool_v = json_to_parameter_value(json(true),
    static_cast<std::uint8_t>(ParameterType::PARAMETER_BOOL));
  EXPECT_EQ(bool_v.get_type(), ParameterType::PARAMETER_BOOL);
  EXPECT_TRUE(bool_v.get<bool>());

  const auto int_v = json_to_parameter_value(json(42),
    static_cast<std::uint8_t>(ParameterType::PARAMETER_INTEGER));
  EXPECT_EQ(int_v.get_type(), ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(int_v.get<int64_t>(), 42);

  // Whole-number JSON float fits an integer slot.
  const auto int_from_float = json_to_parameter_value(json(7.0),
    static_cast<std::uint8_t>(ParameterType::PARAMETER_INTEGER));
  EXPECT_EQ(int_from_float.get<int64_t>(), 7);

  const auto dbl_v = json_to_parameter_value(json(0.125),
    static_cast<std::uint8_t>(ParameterType::PARAMETER_DOUBLE));
  EXPECT_DOUBLE_EQ(dbl_v.get<double>(), 0.125);

  // Integer JSON value fits a double slot.
  const auto dbl_from_int = json_to_parameter_value(json(3),
    static_cast<std::uint8_t>(ParameterType::PARAMETER_DOUBLE));
  EXPECT_DOUBLE_EQ(dbl_from_int.get<double>(), 3.0);

  const auto str_v = json_to_parameter_value(json("car"),
    static_cast<std::uint8_t>(ParameterType::PARAMETER_STRING));
  EXPECT_EQ(str_v.get<std::string>(), "car");
}

TEST(JsonToParameterValue, RejectsNonNumericInteger)
{
  EXPECT_THROW(
    json_to_parameter_value(json("abc"),
      static_cast<std::uint8_t>(ParameterType::PARAMETER_INTEGER)),
    std::invalid_argument);
}

TEST(JsonToParameterValue, RejectsFractionalInteger)
{
  EXPECT_THROW(
    json_to_parameter_value(json(1.5),
      static_cast<std::uint8_t>(ParameterType::PARAMETER_INTEGER)),
    std::invalid_argument);
}

TEST(JsonToParameterValue, RejectsFloatOutsideIntegerRange)
{
  // Whole-number doubles outside [LLONG_MIN, LLONG_MAX+1) must not be
  // silently cast: that is undefined behaviour.
  const auto too_large = json(1.0e20);
  EXPECT_THROW(
    json_to_parameter_value(too_large,
      static_cast<std::uint8_t>(ParameterType::PARAMETER_INTEGER)),
    std::invalid_argument);

  const auto too_negative = json(-1.0e20);
  EXPECT_THROW(
    json_to_parameter_value(too_negative,
      static_cast<std::uint8_t>(ParameterType::PARAMETER_INTEGER)),
    std::invalid_argument);
}

TEST(JsonToParameterValue, ConvertsArrays)
{
  const auto arr = json::array({1.0, 2.0, 3.5});
  const auto val = json_to_parameter_value(arr,
    static_cast<std::uint8_t>(ParameterType::PARAMETER_DOUBLE_ARRAY));
  ASSERT_EQ(val.get_type(), ParameterType::PARAMETER_DOUBLE_ARRAY);
  const auto out = val.get<std::vector<double>>();
  ASSERT_EQ(out.size(), 3u);
  EXPECT_DOUBLE_EQ(out[2], 3.5);
}

TEST(JsonToParameterValue, RejectsByteArray)
{
  EXPECT_THROW(
    json_to_parameter_value(json::array({1, 2}),
      static_cast<std::uint8_t>(ParameterType::PARAMETER_BYTE_ARRAY)),
    std::invalid_argument);
}

TEST(JsonRoundTrip, ScalarParameterRoundTrips)
{
  const Parameter original("voxel_leaf_size", 0.2);
  const auto j = parameter_to_json(original);
  const auto type_id = static_cast<std::uint8_t>(original.get_type());
  const auto reread = json_to_parameter_value(j["value"], type_id);
  EXPECT_EQ(reread.get_type(), original.get_type());
  EXPECT_DOUBLE_EQ(reread.get<double>(), 0.2);
}
