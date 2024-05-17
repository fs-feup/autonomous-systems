#pragma once

#include "adapter_ekf_state_est/adapter.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"

class SENode;

class EufsAdapter : public Adapter {
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr
      _perception_detections_subscription_;  ///< Subscriber for simulated perception detections
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr
      _eufs_wheel_speeds_subscription_;  ///< Subscriber for wheel speeds and steering angle
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr eufs_state_subscription_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

public:
  explicit EufsAdapter(std::shared_ptr<SENode> se_node);

  void mission_state_callback(const eufs_msgs::msg::CanState& msg) const;
  void finish() final;

  void perception_detections_subscription_callback(
      const eufs_msgs::msg::ConeArrayWithCovariance& msg) const;
  void wheel_speeds_subscription_callback(const eufs_msgs::msg::WheelSpeedsStamped& msg) const;
};
