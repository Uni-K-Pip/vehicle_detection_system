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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <httplib.h>  // NOLINT(build/include_order)  cpplint は .h で C system と誤判定
#include <nlohmann/json.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vehicle_detection/parameter_json.hpp"
#include "vehicle_detection/parameter_validation.hpp"

namespace vehicle_detection
{

namespace
{

using nlohmann::json;
using std::chrono::milliseconds;

constexpr const char * kPackageName = "vehicle_detection";
constexpr const char * kGuiFileName = "parameter_gui.html";

std::string read_text_file(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::ostringstream oss;
    oss << "failed to open Web GUI file: " << path.string();
    throw std::runtime_error(oss.str());
  }
  std::ostringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

std::filesystem::path resolve_web_root(const std::string & configured)
{
  if (!configured.empty()) {
    return std::filesystem::path{configured};
  }
  // Resolve relative to the installed share directory.
  const auto share_dir = ament_index_cpp::get_package_share_directory(kPackageName);
  return std::filesystem::path{share_dir} / "web";
}

}  // namespace

class ParameterBridgeNode : public rclcpp::Node
{
public:
  explicit ParameterBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("parameter_bridge_node", options)
  {
    gui_port_ = static_cast<int>(declare_parameter<int>("gui_port", 8081));
    // Bind to loopback by default so the unauthenticated parameter-write API
    // is not exposed beyond the local machine. Override with 0.0.0.0 (or a
    // specific interface) when running inside Docker and exposing the port.
    host_ = declare_parameter<std::string>("host", "127.0.0.1");
    target_nodes_ = declare_parameter<std::vector<std::string>>(
      "target_nodes",
      std::vector<std::string>{
        "/pcd_loader_node",
        "/vehicle_detector_node",
        "/detection_sender_node",
      });
    web_root_override_ = declare_parameter<std::string>("web_root", "");
    service_timeout_ms_ = static_cast<int>(
      declare_parameter<int>("service_timeout_ms", 1500));
    service_ready_timeout_ms_ = static_cast<int>(
      declare_parameter<int>("service_ready_timeout_ms", 200));

    enforce_parameters();

    web_root_ = resolve_web_root(web_root_override_);
    gui_html_ = read_text_file(web_root_ / kGuiFileName);

    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    for (const auto & node_name : target_nodes_) {
      auto client = std::make_shared<rclcpp::AsyncParametersClient>(
        get_node_base_interface(),
        get_node_topics_interface(),
        get_node_graph_interface(),
        get_node_services_interface(),
        node_name,
        rmw_qos_profile_parameters,
        callback_group_);
      param_clients_.emplace(node_name, std::move(client));
    }

    register_routes();
    log_startup_banner();
  }

  ~ParameterBridgeNode() override
  {
    stop_server();
  }

  void start_server()
  {
    if (server_running_.exchange(true)) {
      return;
    }
    server_thread_ = std::thread([this]() {
          RCLCPP_INFO(get_logger(),
        "Web GUI listening on http://%s:%d (web_root=%s)",
        host_.c_str(), gui_port_, web_root_.string().c_str());
          const bool ok = server_.listen(host_.c_str(), gui_port_);
          if (!ok) {
            RCLCPP_ERROR(get_logger(),
          "Web GUI server stopped: failed to bind %s:%d",
          host_.c_str(), gui_port_);
          }
          server_running_ = false;
    });
  }

  void stop_server()
  {
    if (server_.is_running()) {
      server_.stop();
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  void enforce_parameters()
  {
    const auto checks = {
      validate_non_empty_string("host", host_),
      validate_int_min("gui_port", gui_port_, 1),
      validate_int_min("service_timeout_ms", service_timeout_ms_, 1),
      validate_int_min("service_ready_timeout_ms", service_ready_timeout_ms_, 0),
    };
    for (const auto & r : checks) {
      if (!r.ok) {
        RCLCPP_ERROR(get_logger(), "%s", r.reason.c_str());
        throw std::invalid_argument(r.reason);
      }
    }
    if (gui_port_ > 65535) {
      const std::string reason =
        "parameter 'gui_port' must be <= 65535 (got " +
        std::to_string(gui_port_) + ")";
      RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
      throw std::invalid_argument(reason);
    }
    if (target_nodes_.empty()) {
      const std::string reason =
        "parameter 'target_nodes' must contain at least one node name";
      RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
      throw std::invalid_argument(reason);
    }
  }

  void log_startup_banner()
  {
    std::ostringstream oss;
    for (size_t i = 0; i < target_nodes_.size(); ++i) {
      if (i > 0) {oss << ", ";}
      oss << target_nodes_[i];
    }
    RCLCPP_INFO(get_logger(),
      "parameter_bridge_node ready (port=%d, target_nodes=[%s], web_root=%s)",
      gui_port_, oss.str().c_str(), web_root_.string().c_str());
  }

  void register_routes()
  {
    server_.set_default_headers({
        {"Cache-Control", "no-store"},
    });

    server_.Get("/", [this](const httplib::Request &, httplib::Response & res) {
        res.set_content(gui_html_, "text/html; charset=utf-8");
    });

    server_.Get("/api/health",
      [this](const httplib::Request &, httplib::Response & res) {
        json body = {
          {"ok", true},
          {"node", std::string{get_name()}},
          {"target_nodes", target_nodes_},
        };
        res.set_content(body.dump(), "application/json");
      });

    server_.Get("/api/parameters",
      [this](const httplib::Request &, httplib::Response & res) {
        handle_get_parameters(res);
      });

    server_.Post("/api/parameters",
      [this](const httplib::Request & req, httplib::Response & res) {
        handle_post_parameters(req, res);
      });

    server_.set_exception_handler(
      [this](const httplib::Request &, httplib::Response & res,
      std::exception_ptr ep) {
        std::string detail;
        try {
          if (ep) {std::rethrow_exception(ep);}
        } catch (const std::exception & e) {
          detail = e.what();
        } catch (...) {
          detail = "unknown exception";
        }
        RCLCPP_ERROR(get_logger(),
          "GUI request handler raised: %s", detail.c_str());
        json body = {{"ok", false}, {"error", detail}};
        res.status = 500;
        res.set_content(body.dump(), "application/json");
      });
  }

  void handle_get_parameters(httplib::Response & res)
  {
    json nodes = json::array();
    for (const auto & node_name : target_nodes_) {
      nodes.push_back(snapshot_node(node_name));
    }
    json body = {{"ok", true}, {"nodes", std::move(nodes)}};
    res.set_content(body.dump(), "application/json");
  }

  json snapshot_node(const std::string & node_name)
  {
    json out = {
      {"name", node_name},
      {"available", false},
      {"parameters", json::array()},
    };

    auto it = param_clients_.find(node_name);
    if (it == param_clients_.end()) {
      out["error"] = "no client";
      return out;
    }
    auto & client = it->second;

    if (!client->wait_for_service(milliseconds(service_ready_timeout_ms_))) {
      out["error"] = "parameter service not available";
      return out;
    }

    try {
      auto list_future = client->list_parameters({}, 0);
      if (list_future.wait_for(milliseconds(service_timeout_ms_)) !=
        std::future_status::ready)
      {
        out["error"] = "list_parameters timed out";
        return out;
      }
      auto names = list_future.get().names;
      if (names.empty()) {
        out["available"] = true;
        return out;
      }

      auto get_future = client->get_parameters(names);
      if (get_future.wait_for(milliseconds(service_timeout_ms_)) !=
        std::future_status::ready)
      {
        out["error"] = "get_parameters timed out";
        return out;
      }
      auto values = get_future.get();

      auto desc_future = client->describe_parameters(names);
      std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
      if (desc_future.wait_for(milliseconds(service_timeout_ms_)) ==
        std::future_status::ready)
      {
        descriptors = desc_future.get();
      }

      json params = json::array();
      for (size_t i = 0; i < values.size(); ++i) {
        json entry = parameter_to_json(values[i]);
        if (i < descriptors.size()) {
          const auto & d = descriptors[i];
          if (!d.description.empty()) {
            entry["description"] = d.description;
          }
          if (!d.additional_constraints.empty()) {
            entry["constraints"] = d.additional_constraints;
          }
          entry["read_only"] = d.read_only;
        }
        params.push_back(std::move(entry));
      }

      out["available"] = true;
      out["parameters"] = std::move(params);
    } catch (const std::exception & e) {
      out["error"] = e.what();
    }

    return out;
  }

  void handle_post_parameters(
    const httplib::Request & req, httplib::Response & res)
  {
    json payload;
    try {
      payload = json::parse(req.body);
    } catch (const std::exception & e) {
      respond_error(res, 400,
        std::string{"invalid JSON: "} + e.what());
      return;
    }

    if (!payload.is_object() ||
      !payload.contains("node") || !payload["node"].is_string() ||
      !payload.contains("parameters") || !payload["parameters"].is_object())
    {
      respond_error(res, 400,
        "expected JSON of the form {\"node\": \"/...\", \"parameters\": {...}}");
      return;
    }

    const auto target = payload["node"].get<std::string>();
    auto it = param_clients_.find(target);
    if (it == param_clients_.end()) {
      respond_error(res, 503,
        std::string{"node '"} + target + "' is not in target_nodes");
      return;
    }
    auto & client = it->second;
    if (!client->wait_for_service(milliseconds(service_ready_timeout_ms_))) {
      respond_error(res, 503,
        std::string{"parameter service for '"} + target +
        "' is not available");
      return;
    }

    const auto & params_obj = payload["parameters"];
    std::vector<std::string> names;
    names.reserve(params_obj.size());
    for (const auto & item : params_obj.items()) {
      names.push_back(item.key());
    }
    if (names.empty()) {
      respond_error(res, 400, "'parameters' object is empty");
      return;
    }

    json updated = json::array();
    json rejected = json::array();
    bool overall_ok = true;

    std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
    try {
      auto desc_future = client->describe_parameters(names);
      if (desc_future.wait_for(milliseconds(service_timeout_ms_)) !=
        std::future_status::ready)
      {
        respond_error(res, 504,
          "describe_parameters timed out for '" + target + "'");
        return;
      }
      descriptors = desc_future.get();
    } catch (const std::exception & e) {
      respond_error(res, 502,
        std::string{"describe_parameters failed: "} + e.what());
      return;
    }

    std::vector<rclcpp::Parameter> to_set;
    to_set.reserve(names.size());
    std::vector<std::string> to_set_names;
    to_set_names.reserve(names.size());

    for (size_t i = 0; i < names.size(); ++i) {
      const auto & name = names[i];
      const auto & raw = params_obj.at(name);
      if (i >= descriptors.size() ||
        descriptors[i].type ==
        static_cast<uint8_t>(rclcpp::ParameterType::PARAMETER_NOT_SET))
      {
        rejected.push_back({
            {"name", name},
            {"reason", "parameter is not declared on the target node"},
        });
        overall_ok = false;
        continue;
      }
      try {
        auto value = json_to_parameter_value(raw, descriptors[i].type);
        to_set.emplace_back(name, value);
        to_set_names.push_back(name);
      } catch (const std::exception & e) {
        rejected.push_back({
            {"name", name},
            {"reason", e.what()},
        });
        overall_ok = false;
      }
    }

    if (!to_set.empty()) {
      try {
        auto set_future = client->set_parameters(to_set);
        if (set_future.wait_for(milliseconds(service_timeout_ms_)) !=
          std::future_status::ready)
        {
          respond_error(res, 504,
            "set_parameters timed out for '" + target + "'");
          return;
        }
        const auto results = set_future.get();
        for (size_t i = 0; i < results.size() && i < to_set_names.size(); ++i) {
          const auto & r = results[i];
          if (r.successful) {
            updated.push_back(to_set_names[i]);
            RCLCPP_INFO(get_logger(),
              "GUI updated %s.%s = %s",
              target.c_str(), to_set_names[i].c_str(),
              params_obj.at(to_set_names[i]).dump().c_str());
          } else {
            rejected.push_back({
                {"name", to_set_names[i]},
                {"reason", r.reason.empty() ? std::string{"rejected by node"} : r.reason},
            });
            overall_ok = false;
          }
        }
      } catch (const std::exception & e) {
        respond_error(res, 502,
          std::string{"set_parameters failed: "} + e.what());
        return;
      }
    }

    json body = {
      {"ok", overall_ok},
      {"node", target},
      {"updated", std::move(updated)},
      {"rejected", std::move(rejected)},
    };
    res.set_content(body.dump(), "application/json");
  }

  void respond_error(httplib::Response & res, int status, const std::string & msg)
  {
    json body = {{"ok", false}, {"error", msg}};
    res.status = status;
    res.set_content(body.dump(), "application/json");
    RCLCPP_WARN(get_logger(), "GUI %d: %s", status, msg.c_str());
  }

  int gui_port_{8081};
  std::string host_{"0.0.0.0"};
  std::vector<std::string> target_nodes_;
  std::string web_root_override_;
  std::filesystem::path web_root_;
  std::string gui_html_;
  int service_timeout_ms_{1500};
  int service_ready_timeout_ms_{200};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unordered_map<std::string, rclcpp::AsyncParametersClient::SharedPtr>
  param_clients_;

  httplib::Server server_;
  std::thread server_thread_;
  std::atomic<bool> server_running_{false};
};

}  // namespace vehicle_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<vehicle_detection::ParameterBridgeNode> node;
  try {
    node = std::make_shared<vehicle_detection::ParameterBridgeNode>(
      rclcpp::NodeOptions{});
    node->start_server();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("parameter_bridge_node"),
      "fatal during init: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("parameter_bridge_node"),
      "executor stopped: %s", e.what());
  }

  node->stop_server();
  rclcpp::shutdown();
  return 0;
}
