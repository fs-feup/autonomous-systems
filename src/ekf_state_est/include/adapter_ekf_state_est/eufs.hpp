#pragma once

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "ros_node/se_node.hpp"

class EufsAdapter : public SENode {
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr
      _perception_detections_subscription_;  ///< Subscriber for simulated perception detections
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr
      _eufs_wheel_speeds_subscription_;  ///< Subscriber for wheel speeds and steering angle
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr eufs_state_subscription_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

public:
  explicit EufsAdapter(bool use_odometry, bool use_simulated_perception,
                       std::string motion_model_name, std::string data_assocation_model_name,
                       float sml_da_curvature, float sml_initial_limit, float observation_noise,
                       float wheel_speed_sensor_noise, float data_association_limit_distance);

  void mission_state_callback(const eufs_msgs::msg::CanState& msg);
  void finish() final;

  void perception_detections_subscription_callback(
      const eufs_msgs::msg::ConeArrayWithCovariance& msg);
  void wheel_speeds_subscription_callback(const eufs_msgs::msg::WheelSpeedsStamped& msg);
};
