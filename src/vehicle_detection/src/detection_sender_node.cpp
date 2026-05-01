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

#include <inttypes.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "vehicle_detection/detection_json.hpp"
#include "vehicle_detection/http_client.hpp"
#include "vehicle_detection/parameter_validation.hpp"

namespace vehicle_detection
{

namespace
{

constexpr char kSendModeDisabled[] = "disabled";
constexpr char kSendModeRosTopic[] = "ros_topic";
constexpr char kSendModeHttp[] = "http";
constexpr char kSendModeBoth[] = "both";
constexpr std::size_t kMaxQueueSize = 32;

bool is_known_send_mode(const std::string & mode)
{
  return mode == kSendModeDisabled || mode == kSendModeRosTopic ||
         mode == kSendModeHttp || mode == kSendModeBoth;
}

bool send_mode_uses_topic(const std::string & mode)
{
  return mode == kSendModeRosTopic || mode == kSendModeBoth;
}

bool send_mode_uses_http(const std::string & mode)
{
  return mode == kSendModeHttp || mode == kSendModeBoth;
}

}  // namespace

class DetectionSenderNode : public rclcpp::Node
{
public:
  explicit DetectionSenderNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("detection_sender_node", options),
    stop_flag_(false)
  {
    declare_parameters_with_defaults();
    snapshot_ = build_snapshot_from_parameters();
    enforce_initial_parameters();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    detections_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      get_string_param("detections_topic"), qos);

    raw_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      get_string_param("raw_detections_topic"),
      qos,
      std::bind(
        &DetectionSenderNode::on_detections, this, std::placeholders::_1));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &DetectionSenderNode::on_parameter_change, this,
        std::placeholders::_1));

    ensure_http_worker_started(snapshot_.send_mode);

    log_startup_summary();
  }

  ~DetectionSenderNode() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_flag_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct Snapshot
  {
    std::string send_mode;
    std::string http_endpoint_url;
    HttpUrl http_url;
    bool http_url_valid;
    std::int64_t http_timeout_ms;
    std::int64_t http_retry_count;
    std::string http_auth_type;
  };

  void declare_parameters_with_defaults()
  {
    declare_parameter<std::string>(
      "raw_detections_topic", "/vehicle_detections/raw");
    declare_parameter<std::string>("detections_topic", "/vehicle_detections");
    declare_parameter<std::string>("send_mode", "ros_topic");
    declare_parameter<std::string>(
      "http_endpoint_url", "http://host.docker.internal:8080/detections");
    declare_parameter<std::int64_t>("http_timeout_ms", 1000);
    declare_parameter<std::int64_t>("http_retry_count", 0);
    declare_parameter<std::string>("http_auth_type", "none");
  }

  Snapshot build_snapshot_from_parameters() const
  {
    Snapshot s;
    s.send_mode = get_parameter("send_mode").as_string();
    s.http_endpoint_url = get_parameter("http_endpoint_url").as_string();
    s.http_url_valid = parse_http_url(s.http_endpoint_url, &s.http_url);
    s.http_timeout_ms = get_parameter("http_timeout_ms").as_int();
    s.http_retry_count = get_parameter("http_retry_count").as_int();
    s.http_auth_type = get_parameter("http_auth_type").as_string();
    return s;
  }

  void enforce_initial_parameters() const
  {
    if (!is_known_send_mode(snapshot_.send_mode)) {
      throw std::invalid_argument(
              "send_mode must be one of: disabled, ros_topic, http, both");
    }
    const auto checks = {
      validate_int_min("http_timeout_ms", snapshot_.http_timeout_ms, 1),
      validate_int_min("http_retry_count", snapshot_.http_retry_count, 0),
    };
    for (const auto & r : checks) {
      if (!r.ok) {
        throw std::invalid_argument(r.reason);
      }
    }
    if (send_mode_uses_http(snapshot_.send_mode) && !snapshot_.http_url_valid) {
      throw std::invalid_argument(
              "http_endpoint_url must be a valid http:// URL");
    }
  }

  std::string get_string_param(const std::string & name) const
  {
    return get_parameter(name).as_string();
  }

  void log_startup_summary() const
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    RCLCPP_INFO(
      get_logger(),
      "detection_sender_node ready: mode='%s', endpoint='%s', "
      "timeout=%" PRId64 " ms, retries=%" PRId64,
      snapshot_.send_mode.c_str(),
      snapshot_.http_endpoint_url.c_str(),
      snapshot_.http_timeout_ms,
      snapshot_.http_retry_count);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    Snapshot candidate;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      candidate = snapshot_;
    }

    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "send_mode") {
        candidate.send_mode = p.as_string();
      } else if (name == "http_endpoint_url") {
        candidate.http_endpoint_url = p.as_string();
        candidate.http_url_valid =
          parse_http_url(candidate.http_endpoint_url, &candidate.http_url);
      } else if (name == "http_timeout_ms") {
        candidate.http_timeout_ms = p.as_int();
      } else if (name == "http_retry_count") {
        candidate.http_retry_count = p.as_int();
      } else if (name == "http_auth_type") {
        candidate.http_auth_type = p.as_string();
      }
    }

    if (!is_known_send_mode(candidate.send_mode)) {
      result.successful = false;
      result.reason =
        "send_mode must be one of: disabled, ros_topic, http, both";
      RCLCPP_WARN(get_logger(), "rejected parameter update: %s", result.reason.c_str());
      return result;
    }
    const auto checks = {
      validate_int_min("http_timeout_ms", candidate.http_timeout_ms, 1),
      validate_int_min("http_retry_count", candidate.http_retry_count, 0),
    };
    for (const auto & r : checks) {
      if (!r.ok) {
        result.successful = false;
        result.reason = r.reason;
        RCLCPP_WARN(get_logger(), "rejected parameter update: %s", r.reason.c_str());
        return result;
      }
    }
    if (send_mode_uses_http(candidate.send_mode) && !candidate.http_url_valid) {
      result.successful = false;
      result.reason = "http_endpoint_url must be a valid http:// URL";
      RCLCPP_WARN(get_logger(), "rejected parameter update: %s", result.reason.c_str());
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      snapshot_ = candidate;
    }
    ensure_http_worker_started(candidate.send_mode);
    return result;
  }

  void ensure_http_worker_started(const std::string & mode)
  {
    if (!send_mode_uses_http(mode)) {
      return;
    }
    if (worker_.joinable()) {
      return;
    }
    worker_ = std::thread(&DetectionSenderNode::http_worker_loop, this);
  }

  Snapshot copy_snapshot() const
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return snapshot_;
  }

  static DetectionJsonPayload to_json_payload(
    const vision_msgs::msg::Detection3DArray & msg)
  {
    DetectionJsonPayload payload;
    payload.stamp_sec = msg.header.stamp.sec;
    payload.stamp_nanosec = msg.header.stamp.nanosec;
    payload.frame_id = msg.header.frame_id;
    payload.detections.reserve(msg.detections.size());
    for (const auto & det : msg.detections) {
      DetectionJsonItem item;
      item.id = det.id;
      if (det.results.empty()) {
        item.class_label = std::string{};
        item.confidence = 0.0;
      } else {
        item.class_label = det.results.front().hypothesis.class_id;
        item.confidence = det.results.front().hypothesis.score;
      }
      item.center_x = det.bbox.center.position.x;
      item.center_y = det.bbox.center.position.y;
      item.center_z = det.bbox.center.position.z;
      const double dx = det.bbox.size.x;
      const double dy = det.bbox.size.y;
      item.length = std::max(dx, dy);
      item.width = std::min(dx, dy);
      item.height = det.bbox.size.z;
      item.yaw = 0.0;
      payload.detections.push_back(std::move(item));
    }
    return payload;
  }

  void on_detections(
    const vision_msgs::msg::Detection3DArray::SharedPtr msg)
  {
    const Snapshot snap = copy_snapshot();
    if (snap.send_mode == kSendModeDisabled) {
      return;
    }
    if (send_mode_uses_topic(snap.send_mode)) {
      detections_pub_->publish(*msg);
    }
    if (send_mode_uses_http(snap.send_mode)) {
      auto payload = to_json_payload(*msg);
      const std::string body = serialize_detections(payload);
      enqueue_http(body);
    }
  }

  void enqueue_http(std::string body)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.size() >= kMaxQueueSize) {
        queue_.pop_front();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "HTTP queue full; dropping oldest payload");
      }
      queue_.push_back(std::move(body));
    }
    queue_cv_.notify_one();
  }

  void http_worker_loop()
  {
    while (true) {
      std::string body;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] {return stop_flag_ || !queue_.empty();});
        // Exit as soon as shutdown is requested. The queue is already
        // best-effort (oldest-payload-drop on overflow), so we drop any
        // remaining payloads instead of blocking Ctrl+C for
        // (retry_count + 1) * http_timeout_ms per queued POST when the
        // receiver is down.
        if (stop_flag_) {
          queue_.clear();
          return;
        }
        body = std::move(queue_.front());
        queue_.pop_front();
      }
      const Snapshot snap = copy_snapshot();
      if (!send_mode_uses_http(snap.send_mode) || !snap.http_url_valid) {
        continue;
      }
      const auto timeout = std::chrono::milliseconds(snap.http_timeout_ms);
      const std::int64_t total_attempts =
        std::max<std::int64_t>(1, snap.http_retry_count + 1);
      HttpPostResult result;
      for (std::int64_t i = 0; i < total_attempts; ++i) {
        result = http_post_json(snap.http_url, body, timeout);
        if (result.ok) {
          break;
        }
      }
      if (result.ok) {
        RCLCPP_DEBUG(
          get_logger(), "HTTP POST ok (status=%d)", result.status_code);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "HTTP POST failed: status=%d error='%s'",
          result.status_code, result.error.c_str());
      }
    }
  }

  mutable std::mutex snapshot_mutex_;
  Snapshot snapshot_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr raw_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
    detections_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::string> queue_;
  bool stop_flag_;
  std::thread worker_;
};

}  // namespace vehicle_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<vehicle_detection::DetectionSenderNode>(
      rclcpp::NodeOptions{});
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("detection_sender_node"),
      "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
