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

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vehicle_detection/parameter_validation.hpp"
#include "vehicle_detection/point_cloud_processing.hpp"

namespace vehicle_detection
{

namespace
{

constexpr double kMvpDetectionConfidence = 0.8;
constexpr char kMvpDetectionClass[] = "car";

}  // namespace

class VehicleDetectorNode : public rclcpp::Node
{
public:
  explicit VehicleDetectorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("vehicle_detector_node", options)
  {
    declare_parameters_with_defaults();
    snapshot_ = build_snapshot_from_parameters();
    enforce_initial_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    detections_pub_ =
      create_publisher<vision_msgs::msg::Detection3DArray>(
      get_string_param("raw_detections_topic"), qos);
    filtered_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      get_string_param("filtered_points_topic"), qos);
    cluster_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      get_string_param("cluster_markers_topic"), qos);
    vehicle_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      get_string_param("vehicle_markers_topic"), qos);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_string_param("input_points_topic"),
      qos,
      std::bind(&VehicleDetectorNode::on_cloud, this, std::placeholders::_1));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &VehicleDetectorNode::on_parameter_change, this,
        std::placeholders::_1));

    log_startup_summary();
  }

private:
  struct Snapshot
  {
    std::string target_frame_id;
    double voxel_leaf_size;
    RoiBounds roi;
    double ground_distance_threshold;
    EuclideanClusteringParams clustering;
    VehicleSizeLimits vehicle_limits;
  };

  void declare_parameters_with_defaults()
  {
    declare_parameter<std::string>("input_points_topic", "/input/points");
    declare_parameter<std::string>(
      "raw_detections_topic", "/vehicle_detections/raw");
    declare_parameter<std::string>(
      "filtered_points_topic", "/debug/points_filtered");
    declare_parameter<std::string>(
      "cluster_markers_topic", "/debug/clusters");
    declare_parameter<std::string>(
      "vehicle_markers_topic", "/vehicle_markers");
    declare_parameter<std::string>("target_frame_id", "map");
    declare_parameter<double>("voxel_leaf_size", 0.2);
    declare_parameter<double>("roi_min_x", 0.0);
    declare_parameter<double>("roi_max_x", 80.0);
    declare_parameter<double>("roi_min_y", -30.0);
    declare_parameter<double>("roi_max_y", 30.0);
    declare_parameter<double>("roi_min_z", -3.0);
    declare_parameter<double>("roi_max_z", 3.0);
    declare_parameter<double>("ground_distance_threshold", 0.2);
    declare_parameter<double>("cluster_tolerance", 0.8);
    declare_parameter<int64_t>("cluster_min_size", 20);
    declare_parameter<int64_t>("cluster_max_size", 5000);
    declare_parameter<double>("vehicle_min_length", 2.0);
    declare_parameter<double>("vehicle_max_length", 6.0);
    declare_parameter<double>("vehicle_min_width", 1.2);
    declare_parameter<double>("vehicle_max_width", 2.8);
    declare_parameter<double>("vehicle_min_height", 1.0);
    declare_parameter<double>("vehicle_max_height", 3.0);
  }

  Snapshot build_snapshot_from_parameters() const
  {
    Snapshot s;
    s.target_frame_id = get_parameter("target_frame_id").as_string();
    s.voxel_leaf_size = get_parameter("voxel_leaf_size").as_double();
    s.roi = RoiBounds{
      get_parameter("roi_min_x").as_double(),
      get_parameter("roi_min_y").as_double(),
      get_parameter("roi_min_z").as_double(),
      get_parameter("roi_max_x").as_double(),
      get_parameter("roi_max_y").as_double(),
      get_parameter("roi_max_z").as_double(),
    };
    s.ground_distance_threshold =
      get_parameter("ground_distance_threshold").as_double();
    s.clustering = EuclideanClusteringParams{
      get_parameter("cluster_tolerance").as_double(),
      static_cast<std::size_t>(get_parameter("cluster_min_size").as_int()),
      static_cast<std::size_t>(get_parameter("cluster_max_size").as_int()),
    };
    s.vehicle_limits = VehicleSizeLimits{
      get_parameter("vehicle_min_length").as_double(),
      get_parameter("vehicle_max_length").as_double(),
      get_parameter("vehicle_min_width").as_double(),
      get_parameter("vehicle_max_width").as_double(),
      get_parameter("vehicle_min_height").as_double(),
      get_parameter("vehicle_max_height").as_double(),
    };
    return s;
  }

  void enforce_initial_parameters() const
  {
    const auto results = {
      validate_non_empty_string("target_frame_id", snapshot_.target_frame_id),
      validate_positive_double("voxel_leaf_size", snapshot_.voxel_leaf_size),
      validate_positive_double(
        "ground_distance_threshold", snapshot_.ground_distance_threshold),
      validate_positive_double(
        "cluster_tolerance", snapshot_.clustering.tolerance),
      validate_int_min(
        "cluster_min_size",
        static_cast<std::int64_t>(snapshot_.clustering.min_size), 1),
      validate_int_min(
        "cluster_max_size",
        static_cast<std::int64_t>(snapshot_.clustering.max_size),
        static_cast<std::int64_t>(snapshot_.clustering.min_size)),
    };
    for (const auto & r : results) {
      if (!r.ok) {
        throw std::invalid_argument(r.reason);
      }
    }
    if (snapshot_.roi.min_x >= snapshot_.roi.max_x ||
      snapshot_.roi.min_y >= snapshot_.roi.max_y ||
      snapshot_.roi.min_z >= snapshot_.roi.max_z)
    {
      throw std::invalid_argument(
              "ROI bounds must satisfy min_* < max_* on all axes");
    }
    if (snapshot_.vehicle_limits.min_length >=
      snapshot_.vehicle_limits.max_length ||
      snapshot_.vehicle_limits.min_width >=
      snapshot_.vehicle_limits.max_width ||
      snapshot_.vehicle_limits.min_height >=
      snapshot_.vehicle_limits.max_height)
    {
      throw std::invalid_argument(
              "vehicle size bounds must satisfy "
              "min_* < max_* for length/width/height");
    }
  }

  std::string get_string_param(const std::string & name) const
  {
    return get_parameter(name).as_string();
  }

  void log_startup_summary() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(
      get_logger(),
      "vehicle_detector_node ready: target_frame='%s', voxel=%.3f, "
      "ROI x=[%.1f,%.1f] y=[%.1f,%.1f] z=[%.1f,%.1f], "
      "ground_thr=%.3f, cluster_tol=%.2f [%zu,%zu], "
      "vehicle L=[%.1f,%.1f] W=[%.1f,%.1f] H=[%.1f,%.1f]",
      snapshot_.target_frame_id.c_str(),
      snapshot_.voxel_leaf_size,
      snapshot_.roi.min_x, snapshot_.roi.max_x,
      snapshot_.roi.min_y, snapshot_.roi.max_y,
      snapshot_.roi.min_z, snapshot_.roi.max_z,
      snapshot_.ground_distance_threshold,
      snapshot_.clustering.tolerance,
      snapshot_.clustering.min_size,
      snapshot_.clustering.max_size,
      snapshot_.vehicle_limits.min_length, snapshot_.vehicle_limits.max_length,
      snapshot_.vehicle_limits.min_width, snapshot_.vehicle_limits.max_width,
      snapshot_.vehicle_limits.min_height, snapshot_.vehicle_limits.max_height);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    Snapshot candidate;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      candidate = snapshot_;
    }

    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "target_frame_id") {
        candidate.target_frame_id = p.as_string();
      } else if (name == "voxel_leaf_size") {
        candidate.voxel_leaf_size = p.as_double();
      } else if (name == "roi_min_x") {
        candidate.roi.min_x = p.as_double();
      } else if (name == "roi_max_x") {
        candidate.roi.max_x = p.as_double();
      } else if (name == "roi_min_y") {
        candidate.roi.min_y = p.as_double();
      } else if (name == "roi_max_y") {
        candidate.roi.max_y = p.as_double();
      } else if (name == "roi_min_z") {
        candidate.roi.min_z = p.as_double();
      } else if (name == "roi_max_z") {
        candidate.roi.max_z = p.as_double();
      } else if (name == "ground_distance_threshold") {
        candidate.ground_distance_threshold = p.as_double();
      } else if (name == "cluster_tolerance") {
        candidate.clustering.tolerance = p.as_double();
      } else if (name == "cluster_min_size") {
        candidate.clustering.min_size =
          static_cast<std::size_t>(p.as_int());
      } else if (name == "cluster_max_size") {
        candidate.clustering.max_size =
          static_cast<std::size_t>(p.as_int());
      } else if (name == "vehicle_min_length") {
        candidate.vehicle_limits.min_length = p.as_double();
      } else if (name == "vehicle_max_length") {
        candidate.vehicle_limits.max_length = p.as_double();
      } else if (name == "vehicle_min_width") {
        candidate.vehicle_limits.min_width = p.as_double();
      } else if (name == "vehicle_max_width") {
        candidate.vehicle_limits.max_width = p.as_double();
      } else if (name == "vehicle_min_height") {
        candidate.vehicle_limits.min_height = p.as_double();
      } else if (name == "vehicle_max_height") {
        candidate.vehicle_limits.max_height = p.as_double();
      }
    }

    const auto checks = {
      validate_non_empty_string("target_frame_id", candidate.target_frame_id),
      validate_positive_double("voxel_leaf_size", candidate.voxel_leaf_size),
      validate_positive_double(
        "ground_distance_threshold", candidate.ground_distance_threshold),
      validate_positive_double(
        "cluster_tolerance", candidate.clustering.tolerance),
      validate_int_min(
        "cluster_min_size",
        static_cast<std::int64_t>(candidate.clustering.min_size), 1),
      validate_int_min(
        "cluster_max_size",
        static_cast<std::int64_t>(candidate.clustering.max_size),
        static_cast<std::int64_t>(candidate.clustering.min_size)),
    };
    for (const auto & r : checks) {
      if (!r.ok) {
        result.successful = false;
        result.reason = r.reason;
        RCLCPP_WARN(get_logger(), "rejected parameter update: %s", r.reason.c_str());
        return result;
      }
    }
    if (candidate.roi.min_x >= candidate.roi.max_x ||
      candidate.roi.min_y >= candidate.roi.max_y ||
      candidate.roi.min_z >= candidate.roi.max_z)
    {
      result.successful = false;
      result.reason = "ROI bounds must satisfy min_* < max_* on all axes";
      RCLCPP_WARN(get_logger(), "rejected parameter update: %s", result.reason.c_str());
      return result;
    }
    if (candidate.vehicle_limits.min_length >=
      candidate.vehicle_limits.max_length ||
      candidate.vehicle_limits.min_width >=
      candidate.vehicle_limits.max_width ||
      candidate.vehicle_limits.min_height >=
      candidate.vehicle_limits.max_height)
    {
      result.successful = false;
      result.reason =
        "vehicle size bounds must satisfy "
        "min_* < max_* for length/width/height";
      RCLCPP_WARN(get_logger(), "rejected parameter update: %s", result.reason.c_str());
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_ = candidate;
    }
    return result;
  }

  Snapshot copy_snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto t_start = std::chrono::steady_clock::now();
    const Snapshot snap = copy_snapshot();

    sensor_msgs::msg::PointCloud2 transformed_msg;
    if (msg->header.frame_id == snap.target_frame_id) {
      transformed_msg = *msg;
    } else {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          snap.target_frame_id, msg->header.frame_id,
          msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        tf2::doTransform(*msg, transformed_msg, tf);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          get_logger(),
          "TF lookup '%s' -> '%s' failed: %s",
          msg->header.frame_id.c_str(),
          snap.target_frame_id.c_str(), ex.what());
        return;
      }
    }
    transformed_msg.header.frame_id = snap.target_frame_id;

    auto cloud_xyz = std::make_shared<PointCloudXYZ>();
    pcl::fromROSMsg(transformed_msg, *cloud_xyz);
    const std::size_t n_input = cloud_xyz->size();

    auto downsampled = voxel_downsample(cloud_xyz, snap.voxel_leaf_size);
    auto cropped = crop_roi(downsampled, snap.roi);
    auto ground_result = remove_ground_plane(
      cropped, snap.ground_distance_threshold, snap.clustering.min_size);
    auto for_clustering = ground_result.non_ground;
    const std::size_t n_filtered =
      for_clustering ? for_clustering->size() : 0;

    publish_filtered(transformed_msg.header, *for_clustering);

    const auto clusters = extract_clusters(for_clustering, snap.clustering);

    publish_cluster_markers(transformed_msg.header, *for_clustering, clusters);

    vision_msgs::msg::Detection3DArray array;
    array.header = transformed_msg.header;

    visualization_msgs::msg::MarkerArray vehicle_markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_all.header = transformed_msg.header;
    delete_all.ns = "vehicles";
    vehicle_markers.markers.push_back(delete_all);

    int detection_id = 0;
    for (const auto & cluster : clusters) {
      const auto box = compute_aabb(*for_clustering, cluster);
      const auto dim = compute_box_dimensions(box);
      if (!is_passenger_vehicle(dim, snap.vehicle_limits)) {
        continue;
      }
      ++detection_id;

      vision_msgs::msg::Detection3D det;
      det.header = transformed_msg.header;
      det.id = std::to_string(detection_id);
      det.bbox.center.position.x = box.cx();
      det.bbox.center.position.y = box.cy();
      det.bbox.center.position.z = box.cz();
      det.bbox.center.orientation.x = 0.0;
      det.bbox.center.orientation.y = 0.0;
      det.bbox.center.orientation.z = 0.0;
      det.bbox.center.orientation.w = 1.0;
      det.bbox.size.x = box.dx();
      det.bbox.size.y = box.dy();
      det.bbox.size.z = box.dz();

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = kMvpDetectionClass;
      hyp.hypothesis.score = kMvpDetectionConfidence;
      hyp.pose.pose = det.bbox.center;
      det.results.push_back(hyp);

      array.detections.push_back(det);
      vehicle_markers.markers.push_back(make_box_marker(
          transformed_msg.header, "vehicles", detection_id, box,
          0.1f, 1.0f, 0.1f, 0.7f));
    }

    detections_pub_->publish(array);
    vehicle_markers_pub_->publish(vehicle_markers);

    const auto t_end = std::chrono::steady_clock::now();
    const auto ms = std::chrono::duration<double, std::milli>(
      t_end - t_start).count();
    RCLCPP_INFO(
      get_logger(),
      "frame: in=%zu, filtered=%zu, clusters=%zu, detections=%d, %.1f ms",
      n_input, n_filtered, clusters.size(), detection_id, ms);
  }

  void publish_filtered(
    const std_msgs::msg::Header & header,
    const PointCloudXYZ & cloud)
  {
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(cloud, out);
    out.header = header;
    filtered_pub_->publish(out);
  }

  void publish_cluster_markers(
    const std_msgs::msg::Header & header,
    const PointCloudXYZ & cloud,
    const std::vector<pcl::PointIndices> & clusters)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_all.header = header;
    delete_all.ns = "clusters";
    array.markers.push_back(delete_all);

    int id = 0;
    for (const auto & cluster : clusters) {
      ++id;
      const auto box = compute_aabb(cloud, cluster);
      array.markers.push_back(make_box_marker(
          header, "clusters", id, box, 0.2f, 0.6f, 1.0f, 0.4f));
    }
    cluster_markers_pub_->publish(array);
  }

  static visualization_msgs::msg::Marker make_box_marker(
    const std_msgs::msg::Header & header,
    const std::string & ns,
    int id,
    const AxisAlignedBox & box,
    float r, float g, float b, float a)
  {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = box.cx();
    m.pose.position.y = box.cy();
    m.pose.position.z = box.cz();
    m.pose.orientation.w = 1.0;
    m.scale.x = std::max(box.dx(), 0.05);
    m.scale.y = std::max(box.dy(), 0.05);
    m.scale.z = std::max(box.dz(), 0.05);
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    m.lifetime = rclcpp::Duration::from_seconds(0.0);
    return m;
  }

  mutable std::mutex mutex_;
  Snapshot snapshot_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
    detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    cluster_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    vehicle_markers_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;
};

}  // namespace vehicle_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<vehicle_detection::VehicleDetectorNode>(
      rclcpp::NodeOptions{});
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("vehicle_detector_node"),
      "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
