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

#ifndef VEHICLE_DETECTION__POINT_CLOUD_PROCESSING_HPP_
#define VEHICLE_DETECTION__POINT_CLOUD_PROCESSING_HPP_

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <vector>

namespace vehicle_detection
{

using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;

struct AxisAlignedBox
{
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;

  double dx() const {return max_x - min_x;}
  double dy() const {return max_y - min_y;}
  double dz() const {return max_z - min_z;}
  double cx() const {return 0.5 * (min_x + max_x);}
  double cy() const {return 0.5 * (min_y + max_y);}
  double cz() const {return 0.5 * (min_z + max_z);}
};

struct VehicleBoxDimensions
{
  double length;
  double width;
  double height;
};

struct VehicleSizeLimits
{
  double min_length;
  double max_length;
  double min_width;
  double max_width;
  double min_height;
  double max_height;
};

struct RoiBounds
{
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
};

struct EuclideanClusteringParams
{
  double tolerance;
  std::size_t min_size;
  std::size_t max_size;
};

struct GroundRemovalResult
{
  PointCloudXYZ::Ptr non_ground;
  PointCloudXYZ::Ptr ground;
  bool succeeded;
  std::size_t inliers;
};

VehicleBoxDimensions compute_box_dimensions(const AxisAlignedBox & box);

bool is_passenger_vehicle(
  const VehicleBoxDimensions & dim,
  const VehicleSizeLimits & limits);

PointCloudXYZ::Ptr voxel_downsample(
  const PointCloudXYZ::ConstPtr & input,
  double leaf_size);

PointCloudXYZ::Ptr crop_roi(
  const PointCloudXYZ::ConstPtr & input,
  const RoiBounds & roi);

GroundRemovalResult remove_ground_plane(
  const PointCloudXYZ::ConstPtr & input,
  double distance_threshold,
  std::size_t skip_threshold);

std::vector<pcl::PointIndices> extract_clusters(
  const PointCloudXYZ::ConstPtr & input,
  const EuclideanClusteringParams & params);

AxisAlignedBox compute_aabb(
  const PointCloudXYZ & cloud,
  const pcl::PointIndices & indices);

}  // namespace vehicle_detection

#endif  // VEHICLE_DETECTION__POINT_CLOUD_PROCESSING_HPP_
