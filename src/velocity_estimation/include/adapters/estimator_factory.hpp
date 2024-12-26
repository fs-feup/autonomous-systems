#pragma once

#include <rclcpp/rclcpp.hpp>

#include "estimators/ekf.hpp"

inline std::shared_ptr<VelocityEstimator> create_estimator(const VEParameters& params) {
  static const std::unordered_map<
      std::string_view, std::function<std::shared_ptr<VelocityEstimator>(const VEParameters&)>>
      adapter_map = {{"ekf", [](const VEParameters& parameters) {
                        return std::make_shared<EKF>(parameters);
                      }}};

  auto it = adapter_map.find(params._estimation_method_);
  if (it != adapter_map.end()) {
    return it->second(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_estimation"),
                 "Velocity estimation method not recognized");
    return nullptr;
  }
}