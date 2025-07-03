#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "node_/node_control.hpp"

class VehicleAdapter : public Control {
private:
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr go_sub_;
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _lidar_odometry_sub_;

  rclcpp::Time _last_lidar_odometry_received_time_ = rclcpp::Time(0);
  bool _first_lidar_odometry_received_ = false;
  nav_msgs::msg::Odometry _last_lidar_odometry_received_;

public:
  explicit VehicleAdapter(const ControlParameters& params);
  void publish_cmd(double acceleration, double steering) override;
  void go_signal_callback(const custom_interfaces::msg::OperationalStatus msg);
  void _lidar_odometry_subscription_callback(const nav_msgs::msg::Odometry& msg);
};
