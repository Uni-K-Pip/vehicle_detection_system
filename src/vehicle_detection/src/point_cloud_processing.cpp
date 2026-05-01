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

#include "vehicle_detection/point_cloud_processing.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <limits>
#include <memory>

namespace vehicle_detection
{

namespace
{

constexpr double kGroundPlaneAxisToleranceRad = 0.26;  // ~15 deg

}  // namespace

VehicleBoxDimensions compute_box_dimensions(const AxisAlignedBox & box)
{
  const double dx = box.dx();
  const double dy = box.dy();
  return VehicleBoxDimensions{
    std::max(dx, dy),
    std::min(dx, dy),
    box.dz(),
  };
}

bool is_passenger_vehicle(
  const VehicleBoxDimensions & dim,
  const VehicleSizeLimits & limits)
{
  return dim.length >= limits.min_length && dim.length <= limits.max_length &&
         dim.width >= limits.min_width && dim.width <= limits.max_width &&
         dim.height >= limits.min_height && dim.height <= limits.max_height;
}

PointCloudXYZ::Ptr voxel_downsample(
  const PointCloudXYZ::ConstPtr & input,
  double leaf_size)
{
  auto out = std::make_shared<PointCloudXYZ>();
  if (!input || input->empty()) {
    return out;
  }
  if (!(leaf_size > 0.0)) {
    *out = *input;
    return out;
  }
  pcl::VoxelGrid<PointXYZ> vg;
  vg.setInputCloud(input);
  const float leaf = static_cast<float>(leaf_size);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
  return out;
}

PointCloudXYZ::Ptr crop_roi(
  const PointCloudXYZ::ConstPtr & input,
  const RoiBounds & roi)
{
  auto out = std::make_shared<PointCloudXYZ>();
  if (!input || input->empty()) {
    return out;
  }
  pcl::CropBox<PointXYZ> crop;
  crop.setInputCloud(input);
  crop.setMin(Eigen::Vector4f(
      static_cast<float>(roi.min_x),
      static_cast<float>(roi.min_y),
      static_cast<float>(roi.min_z),
      1.0f));
  crop.setMax(Eigen::Vector4f(
      static_cast<float>(roi.max_x),
      static_cast<float>(roi.max_y),
      static_cast<float>(roi.max_z),
      1.0f));
  crop.filter(*out);
  return out;
}

GroundRemovalResult remove_ground_plane(
  const PointCloudXYZ::ConstPtr & input,
  double distance_threshold,
  std::size_t skip_threshold)
{
  GroundRemovalResult result{
    std::make_shared<PointCloudXYZ>(),
    std::make_shared<PointCloudXYZ>(),
    false,
    0};
  if (!input || input->empty()) {
    return result;
  }
  if (input->size() < skip_threshold) {
    *result.non_ground = *input;
    return result;
  }

  pcl::SACSegmentation<PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(kGroundPlaneAxisToleranceRad);
  seg.setInputCloud(input);

  auto inliers = std::make_shared<pcl::PointIndices>();
  pcl::ModelCoefficients coeff;
  seg.segment(*inliers, coeff);

  if (inliers->indices.empty()) {
    *result.non_ground = *input;
    return result;
  }

  pcl::ExtractIndices<PointXYZ> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*result.ground);
  extract.setNegative(true);
  extract.filter(*result.non_ground);

  result.succeeded = true;
  result.inliers = inliers->indices.size();
  return result;
}

std::vector<pcl::PointIndices> extract_clusters(
  const PointCloudXYZ::ConstPtr & input,
  const EuclideanClusteringParams & params)
{
  std::vector<pcl::PointIndices> clusters;
  if (!input || input->empty()) {
    return clusters;
  }

  auto tree = std::make_shared<pcl::search::KdTree<PointXYZ>>();
  tree->setInputCloud(input);

  pcl::EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance(params.tolerance);
  ec.setMinClusterSize(static_cast<int>(params.min_size));
  ec.setMaxClusterSize(static_cast<int>(params.max_size));
  ec.setSearchMethod(tree);
  ec.setInputCloud(input);
  ec.extract(clusters);
  return clusters;
}

AxisAlignedBox compute_aabb(
  const PointCloudXYZ & cloud,
  const pcl::PointIndices & indices)
{
  AxisAlignedBox box{
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for (const auto idx : indices.indices) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= cloud.size()) {
      continue;
    }
    const auto & p = cloud[idx];
    box.min_x = std::min(box.min_x, static_cast<double>(p.x));
    box.min_y = std::min(box.min_y, static_cast<double>(p.y));
    box.min_z = std::min(box.min_z, static_cast<double>(p.z));
    box.max_x = std::max(box.max_x, static_cast<double>(p.x));
    box.max_y = std::max(box.max_y, static_cast<double>(p.y));
    box.max_z = std::max(box.max_z, static_cast<double>(p.z));
  }
  return box;
}

}  // namespace vehicle_detection
