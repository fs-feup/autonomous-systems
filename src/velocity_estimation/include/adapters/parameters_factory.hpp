#pragma once

#include <rclcpp/rclcpp.hpp>

#include "adapters/pacsim_adapter.hpp"

void load_adapter_parameters(VEParameters& params) {
  auto adapter_node = std::make_shared<rclcpp::Node>("temporary_ve_adapter");
  params._car_model_ = adapter_node->declare_parameter("car_model", "bicycle");
  params._motion_model_ = adapter_node->declare_parameter("motion_model", "ctra-no-slip");
  params._adapter_ = adapter_node->declare_parameter("adapter", "pacsim");
}

std::shared_ptr<Adapter> create_ve(const VEParameters& params) {
  static const std::unordered_map<std::string_view,
                                  std::function<std::shared_ptr<Adapter>(const VEParameters&)>>
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