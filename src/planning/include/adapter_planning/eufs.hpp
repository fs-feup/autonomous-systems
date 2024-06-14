#pragma once

#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "planning/planning.hpp"

class EufsAdapter : public Planning {
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr eufs_state_subscription_;
  rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr eufs_pose_subscription_;
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr eufs_map_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr eufs_map_publisher_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

public:
  explicit EufsAdapter(const PlanningParameters& params);

  void mission_state_callback(eufs_msgs::msg::CanState msg);
  void set_mission_state(int mission, int state);
  void pose_callback(const eufs_msgs::msg::CarState& msg);
  void map_callback(const eufs_msgs::msg::ConeArrayWithCovariance& msg);
  void finish() override;
};