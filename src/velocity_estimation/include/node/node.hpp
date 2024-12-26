#pragma once

#include "adapters/parameters.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "estimators/estimator.hpp"
#include "rclcpp/rclcpp.hpp"

class VENode : public rclcpp::Node {
protected:
  VEParameters _parameters_;
  std::shared_ptr<VelocityEstimator> _velocity_estimator_;
  rclcpp::Publisher<custom_interfaces::msg::Velocities>::SharedPtr _velocities_pub_;

public:
  VENode(const VEParameters& parameters);
};