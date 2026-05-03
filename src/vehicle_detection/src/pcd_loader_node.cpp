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
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "vehicle_detection/parameter_validation.hpp"
#include "vehicle_detection/pcd_playlist.hpp"

namespace vehicle_detection
{

class PcdLoaderNode : public rclcpp::Node
{
public:
  explicit PcdLoaderNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pcd_loader_node", options)
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "data/pcd/sample.pcd");
    pcd_files_param_ = declare_parameter<std::vector<std::string>>(
      "pcd_files", std::vector<std::string>{});
    pcd_directory_ = declare_parameter<std::string>("pcd_directory", "");
    pcd_glob_ = declare_parameter<std::string>("pcd_glob", "*.pcd");
    loop_ = declare_parameter<bool>("loop", true);
    input_frame_id_ = declare_parameter<std::string>("input_frame_id", "lidar");
    input_points_topic_ =
      declare_parameter<std::string>("input_points_topic", "/input/points");
    publish_once_ = declare_parameter<bool>("publish_once", false);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

    enforce_parameters();

    // Eagerly load the first playlist entry so a corrupt PCD is reported at
    // startup, matching the MVP fail-fast behaviour. Subsequent entries are
    // loaded lazily inside the publish timer.
    load_index(0);
    if (!cached_msg_) {
      const auto reason = std::string{"failed to load first PCD entry: "} +
      playlist_.front();
      throw std::runtime_error(reason);
    }

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
          load_index(0);
          publish_cached();
        });
    } else {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(period);
      RCLCPP_INFO(get_logger(),
        "publishing %zu file(s) at %.3f Hz on '%s' (loop=%s)",
        playlist_.size(), publish_rate_hz_, input_points_topic_.c_str(),
        loop_ ? "true" : "false");
      timer_ = create_wall_timer(period_ns, [this]() {tick_publish();});
    }
  }

private:
  void enforce_parameters()
  {
    const auto checks = {
      validate_non_empty_string("input_frame_id", input_frame_id_),
      validate_non_empty_string("input_points_topic", input_points_topic_),
      validate_non_empty_string("pcd_glob", pcd_glob_),
      validate_positive_double("publish_rate_hz", publish_rate_hz_),
    };
    for (const auto & r : checks) {
      if (!r.ok) {
        RCLCPP_ERROR(get_logger(), "%s", r.reason.c_str());
        throw std::invalid_argument(r.reason);
      }
    }

    PcdPlaylistInputs inputs;
    inputs.pcd_file = pcd_file_;
    inputs.pcd_files = pcd_files_param_;
    inputs.pcd_directory = pcd_directory_;
    inputs.pcd_glob = pcd_glob_;
    auto resolution = resolve_pcd_playlist(inputs);
    if (!resolution.ok) {
      RCLCPP_ERROR(get_logger(), "%s", resolution.reason.c_str());
      throw std::invalid_argument(resolution.reason);
    }
    playlist_ = std::move(resolution.entries);
    playlist_source_ = to_string(resolution.source);

    RCLCPP_INFO(get_logger(),
      "resolved playlist via '%s' with %zu file(s)",
      playlist_source_, playlist_.size());
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

  void load_index(std::size_t idx)
  {
    if (idx >= playlist_.size()) {
      return;
    }
    if (cached_index_ && *cached_index_ == idx && cached_msg_) {
      return;
    }

    const std::string raw_path = playlist_[idx];
    const std::string abs_path = resolve_pcd_path(raw_path).string();

    pcl::PCLPointCloud2 pcl_cloud;
    const int rc = pcl::io::loadPCDFile(abs_path, pcl_cloud);
    if (rc != 0) {
      RCLCPP_ERROR(get_logger(),
        "failed to load PCD '%s' (pcl::io::loadPCDFile returned %d)",
        abs_path.c_str(), rc);
      cached_msg_.reset();
      cached_index_.reset();
      return;
    }

    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::moveFromPCL(pcl_cloud, *msg);
    msg->header.frame_id = input_frame_id_;

    cached_msg_ = std::move(msg);
    cached_index_ = idx;

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
      "loaded PCD #%zu/%zu '%s' (points=%zu, fields=[%s], frame_id='%s')",
      idx + 1, playlist_.size(), abs_path.c_str(), point_count,
      fields.str().c_str(), input_frame_id_.c_str());

    if (point_count == 0) {
      RCLCPP_WARN(get_logger(),
        "PCD '%s' is empty; publishing empty PointCloud2", abs_path.c_str());
    }
  }

  void publish_cached()
  {
    if (!cached_msg_) {
      return;
    }
    cached_msg_->header.stamp = now();
    publisher_->publish(*cached_msg_);
  }

  void tick_publish()
  {
    if (playlist_.empty() || current_index_ >= playlist_.size()) {
      return;
    }

    load_index(current_index_);
    publish_cached();

    const auto published_index = current_index_;
    ++current_index_;
    if (current_index_ >= playlist_.size()) {
      if (loop_) {
        current_index_ = 0;
        if (playlist_.size() > 1) {
          RCLCPP_INFO(get_logger(),
            "reached end of playlist (#%zu); looping back to #1",
            published_index + 1);
        }
      } else {
        if (timer_) {
          timer_->cancel();
        }
        RCLCPP_INFO(get_logger(),
          "playlist finished after #%zu (loop=false); stopped publishing",
          published_index + 1);
      }
    }
  }

  std::string pcd_file_;
  std::vector<std::string> pcd_files_param_;
  std::string pcd_directory_;
  std::string pcd_glob_;
  bool loop_{true};
  std::string input_frame_id_;
  std::string input_points_topic_;
  bool publish_once_{false};
  double publish_rate_hz_{1.0};

  std::vector<std::string> playlist_;
  const char * playlist_source_{"pcd_file"};
  std::size_t current_index_{0};
  std::optional<std::size_t> cached_index_;

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
