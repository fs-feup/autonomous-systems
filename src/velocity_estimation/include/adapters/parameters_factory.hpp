#pragma once

#include <rclcpp/rclcpp.hpp>

#include "adapters/pacsim_adapter.hpp"
/**
 * @brief Loads parameters used to configure the velocity estimation node from a launch file.
 */
void load_adapter_parameters(VEParameters& params) {
  auto adapter_node = std::make_shared<rclcpp::Node>("temporary_ve_adapter");
  params._estimation_method_ = adapter_node->declare_parameter("estimation_method", "ekf");
  params._adapter_ = adapter_node->declare_parameter("adapter", "pacsim");
  params._ekf_process_noise_ = adapter_node->declare_parameter<double>("ekf_process_noise", 0.01);
}

/**
 * @brief Factory function to create a velocity estimation node based on the provided parameters.
 *
 * @param params Parameters used to configure the node.
 * @return std::shared_ptr<VENode> Pointer to the created velocity estimation node.
 */
std::shared_ptr<VENode> create_ve(const VEParameters& params) {
  static const std::unordered_map<std::string_view,
                                  std::function<std::shared_ptr<VENode>(const VEParameters&)>>
      adapter_map = {{"pacsim", [](const VEParameters& parameters) {
                        return std::make_shared<PacsimAdapter>(parameters);
                      }}};

  auto it = adapter_map.find(params._adapter_);
  if (it != adapter_map.end()) {
    return it->second(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_estimation"), "Adapter type not recognized");
    return nullptr;
  }
}