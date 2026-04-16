#pragma once

#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "ros_node/ros_node.hpp"

class InvictaSimAdapter : public ControlNode {
private:
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_pub_;
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vehicle_pose_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr vehicle_velocity_sub_;

public:
  explicit InvictaSimAdapter(const ControlParameters& params);
  void pose_callback(const custom_interfaces::msg::Pose& msg);
  void velocity_callback(const custom_interfaces::msg::Velocities& msg);
  void publish_command(common_lib::structures::ControlCommand cmd) override;
};
