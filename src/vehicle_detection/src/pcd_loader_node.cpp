#include "vehicle_detection/parameter_validation.hpp"

#include <chrono>
#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vehicle_detection
{

class PcdLoaderNode : public rclcpp::Node
{
public:
  explicit PcdLoaderNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pcd_loader_node", options)
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "data/pcd/sample.pcd");
    input_frame_id_ = declare_parameter<std::string>("input_frame_id", "lidar");
    input_points_topic_ =
      declare_parameter<std::string>("input_points_topic", "/input/points");
    publish_once_ = declare_parameter<bool>("publish_once", false);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

    enforce_parameters();
    load_cloud();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      input_points_topic_, qos);

    if (publish_once_) {
      RCLCPP_INFO(get_logger(),
        "publish_once=true; publishing one frame and keeping node alive");
      one_shot_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
          one_shot_timer_->cancel();
          publish_cloud();
        });
    } else {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(period);
      RCLCPP_INFO(get_logger(),
        "publishing at %.3f Hz on '%s'",
        publish_rate_hz_, input_points_topic_.c_str());
      timer_ = create_wall_timer(period_ns, [this]() { publish_cloud(); });
    }
  }

private:
  void enforce_parameters()
  {
    const auto resolved = resolve_pcd_path(pcd_file_);
    pcd_file_ = resolved.string();

    const auto checks = {
      validate_non_empty_string("input_frame_id", input_frame_id_),
      validate_non_empty_string("input_points_topic", input_points_topic_),
      validate_positive_double("publish_rate_hz", publish_rate_hz_),
      validate_existing_file("pcd_file", pcd_file_),
    };
    for (const auto & r : checks) {
      if (!r.ok) {
        RCLCPP_ERROR(get_logger(), "%s", r.reason.c_str());
        throw std::invalid_argument(r.reason);
      }
    }
  }

  static std::filesystem::path resolve_pcd_path(const std::string & raw)
  {
    std::filesystem::path p{raw};
    if (p.is_absolute()) {
      return p;
    }
    std::error_code ec;
    auto abs = std::filesystem::absolute(p, ec);
    if (ec) {
      return p;
    }
    return abs;
  }

  void load_cloud()
  {
    pcl::PCLPointCloud2 pcl_cloud;
    const int rc = pcl::io::loadPCDFile(pcd_file_, pcl_cloud);
    if (rc != 0) {
      std::ostringstream oss;
      oss << "failed to load PCD '" << pcd_file_
          << "' (pcl::io::loadPCDFile returned " << rc << ")";
      RCLCPP_ERROR(get_logger(), "%s", oss.str().c_str());
      throw std::runtime_error(oss.str());
    }

    cached_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::moveFromPCL(pcl_cloud, *cached_msg_);
    cached_msg_->header.frame_id = input_frame_id_;

    const auto point_count =
      static_cast<size_t>(cached_msg_->width) *
      static_cast<size_t>(cached_msg_->height);

    std::ostringstream fields;
    for (size_t i = 0; i < cached_msg_->fields.size(); ++i) {
      if (i > 0) {
        fields << ",";
      }
      fields << cached_msg_->fields[i].name;
    }

    RCLCPP_INFO(get_logger(),
      "loaded PCD '%s' (points=%zu, fields=[%s], frame_id='%s')",
      pcd_file_.c_str(), point_count, fields.str().c_str(),
      input_frame_id_.c_str());

    if (point_count == 0) {
      RCLCPP_WARN(get_logger(),
        "PCD '%s' is empty; publishing empty PointCloud2", pcd_file_.c_str());
    }
  }

  void publish_cloud()
  {
    if (!cached_msg_) {
      return;
    }
    cached_msg_->header.stamp = now();
    publisher_->publish(*cached_msg_);
  }

  std::string pcd_file_;
  std::string input_frame_id_;
  std::string input_points_topic_;
  bool publish_once_{false};
  double publish_rate_hz_{1.0};

  sensor_msgs::msg::PointCloud2::SharedPtr cached_msg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_;
};

}  // namespace vehicle_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<vehicle_detection::PcdLoaderNode>(
      rclcpp::NodeOptions{});
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_loader_node"),
      "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
