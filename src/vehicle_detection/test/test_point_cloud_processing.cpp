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

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "vehicle_detection/point_cloud_processing.hpp"

using vehicle_detection::AxisAlignedBox;
using vehicle_detection::EuclideanClusteringParams;
using vehicle_detection::PointCloudXYZ;
using vehicle_detection::PointXYZ;
using vehicle_detection::RoiBounds;
using vehicle_detection::VehicleBoxDimensions;
using vehicle_detection::VehicleSizeLimits;
using vehicle_detection::compute_aabb;
using vehicle_detection::compute_box_dimensions;
using vehicle_detection::crop_roi;
using vehicle_detection::extract_clusters;
using vehicle_detection::is_passenger_vehicle;

namespace
{

VehicleSizeLimits passenger_car_limits()
{
  return VehicleSizeLimits{2.0, 6.0, 1.2, 2.8, 1.0, 3.0};
}

PointCloudXYZ::Ptr make_axis_aligned_cube(
  double cx, double cy, double cz, double dx, double dy, double dz)
{
  auto cloud = std::make_shared<PointCloudXYZ>();
  const double hx = 0.5 * dx;
  const double hy = 0.5 * dy;
  const double hz = 0.5 * dz;
  for (int ix = 0; ix < 5; ++ix) {
    for (int iy = 0; iy < 5; ++iy) {
      for (int iz = 0; iz < 5; ++iz) {
        PointXYZ p;
        p.x = static_cast<float>(cx - hx + (dx * ix) / 4.0);
        p.y = static_cast<float>(cy - hy + (dy * iy) / 4.0);
        p.z = static_cast<float>(cz - hz + (dz * iz) / 4.0);
        cloud->push_back(p);
      }
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace

TEST(BoxDimensions, LengthIsLargerOfDxDy)
{
  const AxisAlignedBox box{0.0, 0.0, 0.0, 4.5, 1.8, 1.6};
  const auto dim = compute_box_dimensions(box);
  EXPECT_DOUBLE_EQ(dim.length, 4.5);
  EXPECT_DOUBLE_EQ(dim.width, 1.8);
  EXPECT_DOUBLE_EQ(dim.height, 1.6);
}

TEST(BoxDimensions, SwapsWhenDyExceedsDx)
{
  const AxisAlignedBox box{0.0, 0.0, 0.0, 1.8, 4.5, 1.6};
  const auto dim = compute_box_dimensions(box);
  EXPECT_DOUBLE_EQ(dim.length, 4.5);
  EXPECT_DOUBLE_EQ(dim.width, 1.8);
  EXPECT_DOUBLE_EQ(dim.height, 1.6);
}

TEST(BoxDimensions, EqualDxDyKeepsBothValues)
{
  const AxisAlignedBox box{0.0, 0.0, 0.0, 2.0, 2.0, 1.6};
  const auto dim = compute_box_dimensions(box);
  EXPECT_DOUBLE_EQ(dim.length, 2.0);
  EXPECT_DOUBLE_EQ(dim.width, 2.0);
  EXPECT_DOUBLE_EQ(dim.height, 1.6);
}

TEST(VehicleFilter, AcceptsTypicalPassengerCar)
{
  const VehicleBoxDimensions dim{4.5, 1.8, 1.5};
  EXPECT_TRUE(is_passenger_vehicle(dim, passenger_car_limits()));
}

TEST(VehicleFilter, RejectsTooShort)
{
  const VehicleBoxDimensions dim{1.5, 1.8, 1.5};
  EXPECT_FALSE(is_passenger_vehicle(dim, passenger_car_limits()));
}

TEST(VehicleFilter, RejectsTooLong)
{
  const VehicleBoxDimensions dim{8.0, 1.8, 1.5};
  EXPECT_FALSE(is_passenger_vehicle(dim, passenger_car_limits()));
}

TEST(VehicleFilter, RejectsTooNarrow)
{
  const VehicleBoxDimensions dim{4.5, 1.0, 1.5};
  EXPECT_FALSE(is_passenger_vehicle(dim, passenger_car_limits()));
}

TEST(VehicleFilter, RejectsTooWide)
{
  const VehicleBoxDimensions dim{4.5, 3.0, 1.5};
  EXPECT_FALSE(is_passenger_vehicle(dim, passenger_car_limits()));
}

TEST(VehicleFilter, RejectsTooLow)
{
  const VehicleBoxDimensions dim{4.5, 1.8, 0.5};
  EXPECT_FALSE(is_passenger_vehicle(dim, passenger_car_limits()));
}

TEST(VehicleFilter, RejectsTooTall)
{
  const VehicleBoxDimensions dim{4.5, 1.8, 3.5};
  EXPECT_FALSE(is_passenger_vehicle(dim, passenger_car_limits()));
}

TEST(VehicleFilter, AcceptsBoundaryValues)
{
  const auto lim = passenger_car_limits();
  EXPECT_TRUE(is_passenger_vehicle(VehicleBoxDimensions{2.0, 1.2, 1.0}, lim));
  EXPECT_TRUE(is_passenger_vehicle(VehicleBoxDimensions{6.0, 2.8, 3.0}, lim));
}

TEST(ComputeAabb, BoundsMatchKnownCube)
{
  auto cloud = make_axis_aligned_cube(10.0, 0.0, 0.5, 4.0, 2.0, 1.5);
  pcl::PointIndices indices;
  indices.indices.resize(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    indices.indices[i] = static_cast<int>(i);
  }
  const auto box = compute_aabb(*cloud, indices);
  EXPECT_NEAR(box.min_x, 8.0, 1e-5);
  EXPECT_NEAR(box.max_x, 12.0, 1e-5);
  EXPECT_NEAR(box.min_y, -1.0, 1e-5);
  EXPECT_NEAR(box.max_y, 1.0, 1e-5);
  EXPECT_NEAR(box.min_z, -0.25, 1e-5);
  EXPECT_NEAR(box.max_z, 1.25, 1e-5);
  EXPECT_NEAR(box.cx(), 10.0, 1e-5);
  EXPECT_NEAR(box.cy(), 0.0, 1e-5);
  EXPECT_NEAR(box.cz(), 0.5, 1e-5);
  EXPECT_NEAR(box.dx(), 4.0, 1e-5);
  EXPECT_NEAR(box.dy(), 2.0, 1e-5);
  EXPECT_NEAR(box.dz(), 1.5, 1e-5);
}

TEST(ComputeAabb, EmptyIndicesProducesInvertedBox)
{
  PointCloudXYZ cloud;
  pcl::PointIndices indices;
  const auto box = compute_aabb(cloud, indices);
  EXPECT_GT(box.min_x, box.max_x);
}

TEST(CropRoi, KeepsOnlyPointsInsideBounds)
{
  auto cloud = std::make_shared<PointCloudXYZ>();
  cloud->push_back(PointXYZ{0.0f, 0.0f, 0.0f});
  cloud->push_back(PointXYZ{5.0f, 5.0f, 1.0f});
  cloud->push_back(PointXYZ{50.0f, 0.0f, 0.0f});
  cloud->push_back(PointXYZ{0.0f, 50.0f, 0.0f});
  cloud->push_back(PointXYZ{0.0f, 0.0f, 10.0f});
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;

  const RoiBounds roi{-10.0, -10.0, -2.0, 10.0, 10.0, 2.0};
  const auto cropped = crop_roi(cloud, roi);
  ASSERT_NE(cropped, nullptr);
  EXPECT_EQ(cropped->size(), 2u);
}

TEST(CropRoi, EmptyCloudReturnsEmpty)
{
  auto cloud = std::make_shared<PointCloudXYZ>();
  const RoiBounds roi{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0};
  const auto cropped = crop_roi(cloud, roi);
  ASSERT_NE(cropped, nullptr);
  EXPECT_TRUE(cropped->empty());
}

TEST(ExtractClusters, FindsTwoSeparateClusters)
{
  auto cloud = std::make_shared<PointCloudXYZ>();
  for (int i = 0; i < 30; ++i) {
    PointXYZ p;
    p.x = static_cast<float>(0.0 + 0.05 * (i % 5));
    p.y = static_cast<float>(0.0 + 0.05 * ((i / 5) % 5));
    p.z = static_cast<float>(0.0 + 0.05 * (i / 25));
    cloud->push_back(p);
  }
  for (int i = 0; i < 30; ++i) {
    PointXYZ p;
    p.x = static_cast<float>(20.0 + 0.05 * (i % 5));
    p.y = static_cast<float>(20.0 + 0.05 * ((i / 5) % 5));
    p.z = static_cast<float>(0.0 + 0.05 * (i / 25));
    cloud->push_back(p);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;

  const auto clusters =
    extract_clusters(cloud, EuclideanClusteringParams{0.3, 10, 1000});
  EXPECT_EQ(clusters.size(), 2u);
  for (const auto & c : clusters) {
    EXPECT_GE(c.indices.size(), 10u);
  }
}

TEST(ExtractClusters, EmptyCloudYieldsNoClusters)
{
  auto cloud = std::make_shared<PointCloudXYZ>();
  const auto clusters =
    extract_clusters(cloud, EuclideanClusteringParams{0.3, 10, 1000});
  EXPECT_TRUE(clusters.empty());
}
