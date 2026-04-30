#include "vehicle_detection/parameter_validation.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using vehicle_detection::validate_double_range;
using vehicle_detection::validate_enum;
using vehicle_detection::validate_existing_file;
using vehicle_detection::validate_int_min;
using vehicle_detection::validate_non_empty_string;
using vehicle_detection::validate_positive_double;

TEST(ParameterValidation, NonEmptyStringAcceptsValue)
{
  const auto r = validate_non_empty_string("frame_id", "lidar");
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.reason.empty());
}

TEST(ParameterValidation, NonEmptyStringRejectsEmpty)
{
  const auto r = validate_non_empty_string("frame_id", "");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("frame_id"), std::string::npos);
}

TEST(ParameterValidation, PositiveDoubleAcceptsAboveZero)
{
  EXPECT_TRUE(validate_positive_double("rate", 0.1).ok);
  EXPECT_TRUE(validate_positive_double("rate", 1000.0).ok);
}

TEST(ParameterValidation, PositiveDoubleRejectsZeroAndNegative)
{
  EXPECT_FALSE(validate_positive_double("rate", 0.0).ok);
  EXPECT_FALSE(validate_positive_double("rate", -0.5).ok);
}

TEST(ParameterValidation, DoubleRangeBoundariesInclusive)
{
  EXPECT_TRUE(validate_double_range("voxel", 0.05, 0.05, 1.0).ok);
  EXPECT_TRUE(validate_double_range("voxel", 1.0, 0.05, 1.0).ok);
  EXPECT_FALSE(validate_double_range("voxel", 1.01, 0.05, 1.0).ok);
  EXPECT_FALSE(validate_double_range("voxel", 0.0, 0.05, 1.0).ok);
}

TEST(ParameterValidation, IntMinRejectsBelow)
{
  EXPECT_TRUE(validate_int_min("retry", 0, 0).ok);
  EXPECT_TRUE(validate_int_min("retry", 5, 0).ok);
  EXPECT_FALSE(validate_int_min("retry", -1, 0).ok);
}

TEST(ParameterValidation, EnumAcceptsAllowedValue)
{
  EXPECT_TRUE(validate_enum("send_mode", "ros_topic",
    {"disabled", "ros_topic", "http", "both"}).ok);
}

TEST(ParameterValidation, EnumRejectsUnknownValue)
{
  const auto r = validate_enum("send_mode", "udp",
    {"disabled", "ros_topic", "http", "both"});
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("send_mode"), std::string::npos);
  EXPECT_NE(r.reason.find("udp"), std::string::npos);
}

TEST(ParameterValidation, ExistingFileRejectsMissing)
{
  const auto r = validate_existing_file("pcd_file",
    "/this/path/should/not/exist/sample.pcd");
  EXPECT_FALSE(r.ok);
}

TEST(ParameterValidation, ExistingFileAcceptsRealFile)
{
  const auto tmp_dir = std::filesystem::temp_directory_path();
  const auto tmp = tmp_dir / "vehicle_detection_test_param.txt";
  {
    std::ofstream ofs(tmp);
    ofs << "ok";
  }
  const auto r = validate_existing_file("pcd_file", tmp.string());
  std::error_code ec;
  std::filesystem::remove(tmp, ec);
  EXPECT_TRUE(r.ok) << r.reason;
}

TEST(ParameterValidation, ExistingFileRejectsDirectory)
{
  const auto tmp_dir = std::filesystem::temp_directory_path();
  const auto r = validate_existing_file("pcd_file", tmp_dir.string());
  EXPECT_FALSE(r.ok);
}
