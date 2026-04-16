#pragma once

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "planning/planning.hpp"

class InvictaSimAdapter : public Planning {
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_subscription_;

public:
  explicit InvictaSimAdapter(const PlanningParameters& params);

  void pose_callback(const custom_interfaces::msg::Pose& msg);
  void track_callback(const custom_interfaces::msg::ConeArray& msg);
  void finish() override;
};
