#pragma once

#include "adapters/parameters.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "rclcpp/rclcpp.hpp"

class VENode : public rclcpp::Node {
  VEParameters _parameters_;
  rclcpp::Publisher<custom_interfaces::msg::Velocities>::SharedPtr _velocities_pub_;

public:
  VENode(const VEParameters& parameters);
};