#pragma once

#include "adapter_ekf_state_est/adapter.hpp"

class SENode;

class FsdsAdapter : public Adapter {
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _fs_imu_subscription_;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr _fs_wheel_speeds_subscription_;

public:
  explicit FsdsAdapter(std::shared_ptr<SENode> se_node);

  void mission_state_callback(const fs_msgs::msg::GoSignal& msg) const;
  void finish() final;

  void wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates& msg) const;
};