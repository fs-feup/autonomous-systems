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
  auto adapter_node = std::make_shared<rclcpp::Node>("control_adapter");

  params.using_simulated_se_ = adapter_node->declare_parameter<bool>("use_simulated_se");
  params.use_simulated_planning_ = adapter_node->declare_parameter<bool>("use_simulated_planning");
  params.lookahead_gain_ = adapter_node->declare_parameter<double>("lookahead_gain");
  params.pid_kp_ = adapter_node->declare_parameter<double>("pid_kp");
  params.pid_ki_ = adapter_node->declare_parameter<double>("pid_ki");
  params.pid_kd_ = adapter_node->declare_parameter<double>("pid_kd");
  params.pid_tau_ = adapter_node->declare_parameter<double>("pid_tau");
  params.pid_t_ = adapter_node->declare_parameter<double>("pid_t");
  params.pid_lim_min_ = adapter_node->declare_parameter<double>("pid_lim_min");
  params.pid_lim_max_ = adapter_node->declare_parameter<double>("pid_lim_max");
  params.pid_anti_windup_ = adapter_node->declare_parameter<double>("pid_anti_windup");
  std::string adapter_type = adapter_node->declare_parameter<std::string>("adapter");
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