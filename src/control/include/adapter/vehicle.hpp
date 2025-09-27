#pragma once

#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "ros_node/ros_node.hpp"

class VehicleAdapter : public ControlNode {
private:
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr go_sub_;
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_pub_;

public:
  explicit VehicleAdapter(const ControlParameters &params);
  void publish_cmd(double acceleration, double steering) override;
  void go_signal_callback(const custom_interfaces::msg::OperationalStatus msg);
};
