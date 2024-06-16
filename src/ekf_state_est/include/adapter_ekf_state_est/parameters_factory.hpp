#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "adapter_ekf_state_est/eufs.hpp"
#include "adapter_ekf_state_est/fsds.hpp"
#include "adapter_ekf_state_est/pacsim.hpp"
#include "adapter_ekf_state_est/vehicle.hpp"
#include "ros_node/se_node.hpp"

struct EKFStateEstParameters;

std::string load_adapter_parameters(EKFStateEstParameters& params) {
    auto adapter_node = std::make_shared<rclcpp::Node>("ekf_state_est_adapter");
    params.use_odometry_ = adapter_node->declare_parameter("use_odometry", true);
    params.use_simulated_perception_ =
            adapter_node->declare_parameter("use_simulated_perception", false);
    params.motion_model_name_ =
            adapter_node->declare_parameter("motion_model", "normal_velocity_model");
    params.data_assocation_model_name_ =
            adapter_node->declare_parameter("data_assocation_model", "simple_ml");
    params.sml_da_curvature_ =
            static_cast<float>(adapter_node->declare_parameter("sml_da_curvature", 15.0f));
    params.sml_initial_limit_ =
            static_cast<float>(adapter_node->declare_parameter("sml_initial_limit", 0.1f));
    params.observation_noise_ =
            static_cast<float>(adapter_node->declare_parameter("observation_noise", 0.01f));
    params.wheel_speed_sensor_noise_ =
            static_cast<float>(adapter_node->declare_parameter("wheel_speed_sensor_noise", 0.1f));
    params.data_association_limit_distance_ =
            static_cast<float>(adapter_node->declare_parameter("data_association_limit_distance", 71.0f));

    return adapter_node->declare_parameter("adapter", "vehicle");
}

std::shared_ptr<SENode> create_ekf_state_est(const std::string_view& adapter_type,
                                             const EKFStateEstParameters& params) {
  static const std::unordered_map<
      std::string_view, std::function<std::shared_ptr<SENode>(const EKFStateEstParameters&)>>
      adapter_map = {
          {"vehicle",
           [](const EKFStateEstParameters& parameters) {
             return std::make_shared<VehicleAdapter>(parameters);
           }},
          {"pacsim",
           [](const EKFStateEstParameters& parameters) {
             return std::make_shared<PacsimAdapter>(parameters);
           }},
          {"eufs",
           [](const EKFStateEstParameters& parameters) {
             return std::make_shared<EufsAdapter>(parameters);
           }},
          {"fsds",
           [](const EKFStateEstParameters& parameters) {
             return std::make_shared<FsdsAdapter>(parameters);
           }},
      };

  auto it = adapter_map.find(adapter_type);
  if (it != adapter_map.end()) {
    return it->second(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Adapter type not recognized");
    return nullptr;
  }
}