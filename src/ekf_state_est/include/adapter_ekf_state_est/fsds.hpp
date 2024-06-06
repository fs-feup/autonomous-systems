#pragma once

#include "ros_node/se_node.hpp"

class FsdsAdapter : public SENode {
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _fs_imu_subscription_;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr _fs_wheel_speeds_subscription_;

public:
  explicit FsdsAdapter(bool use_odometry, bool use_simulated_perception,
                       std::string motion_model_name, std::string data_assocation_model_name,
                       float sml_da_curvature, float sml_initial_limit, float observation_noise,
                       float wheel_speed_sensor_noise, float data_association_limit_distance);

  void mission_state_callback(const fs_msgs::msg::GoSignal& msg);
  void finish() final;

  void wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates& msg);
};