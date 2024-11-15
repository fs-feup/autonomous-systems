#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
#include "adapter_control/pacsim.hpp"
#include "adapter_control/vehicle.hpp"
#include "node_/node_control.hpp"

struct ControlParameters;

std::string load_adapter_parameters(ControlParameters& params) {
  auto adapter_node = std::make_shared<rclcpp::Node>("control_adapter");<
  params.using_simulated_se_ = adapter_node->declare_parameter("use_simulated_se", true);
  params.use_simulated_planning_ = adapter_node->declare_parameter("use_simulated_planning", true);
  params.lookahead_gain_ = adapter_node->declare_parameter("lookahead_gain", 0.5);
  params.pid_kp_ = adapter_node->declare_parameter("pid_kp", 0.4);
  params.pid_ki_ = adapter_node->declare_parameter("pid_ki", 0.3);
  params.pid_kd_ = adapter_node->declare_parameter("pid_kd", 0.09);load_adapter_parameters
  params.pid_tau_ = adapter_node->declare_parameter("pid_tau", 0.5);
  params.pid_t_ = adapter_node->declare_parameter("pid_t", 0.01);
  params.pid_lim_min_ = adapter_node->declare_parameter("pid_lim_min", -1.0);
  params.pid_lim_max_ = adapter_node->declare_parameter("pid_lim_max", 1.0);
  params.pid_anti_windup_ = adapter_node->declare_parameter("pid_anti_windup", 0.7);
  std::string adapter_type = adapter_node->declare_parameter("adapter", "vehicle");
  params.map_frame_id_ = adapter_type == "eufs" ? "base_footprint" : "map";

  return adapter_type;
}

std::shared_ptr<Control> create_control(const std::string_view& adapter_type,
                                        const ControlParameters& params) {
  static const std::unordered_map<std::string_view,
                                  std::function<std::shared_ptr<Control>(const ControlParameters&)>>
      adapter_map = {
          {"vehicle",
           [](const ControlParameters& parameters) {
             return std::make_shared<VehicleAdapter>(parameters);
           }},
          {"pacsim",
           [](const ControlParameters& parameters) {
             return std::make_shared<PacSimAdapter>(parameters);
           }},
          {"eufs",
           [](const ControlParameters& parameters) {
             return std::make_shared<EufsAdapter>(parameters);
           }},
          {"fsds",
           [](const ControlParameters& parameters) {
             return std::make_shared<FsdsAdapter>(parameters);
           }},
      };

  auto it = adapter_map.find(adapter_type);
  if (it != adapter_map.end()) {
    return it->second(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "Adapter type not recognized");
    return nullptr;
  }
}