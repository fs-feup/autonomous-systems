#pragma once

#include "adapter_ekf_state_est/adapter.hpp"

class SENode;

class EufsAdapter : public Adapter {
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr
      _eufs_wheel_speeds_subscription;
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr eufs_state_subscription_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

 public:
  explicit EufsAdapter(SENode* speed_est);

  void init() override;
  void mission_state_callback(eufs_msgs::msg::CanState msg);
  void finish() override;

  void wheel_speeds_subscription_callback(const eufs_msgs::msg::WheelSpeedsStamped msg);
};
