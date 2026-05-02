#pragma once

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "planning/planning.hpp"
#include "std_msgs/msg/float64.hpp"

class InvictaSimAdapter : public Planning {
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr mission_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vehicle_localization_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lap_counter_sub_;

public:
  explicit InvictaSimAdapter(const PlanningParameters& params);

  void map_callback(const custom_interfaces::msg::ConeArray& msg);

  void pose_callback(const custom_interfaces::msg::Pose& msg);

  void lap_counter_callback(const std_msgs::msg::Float64& msg);

  void mission_state_callback(const custom_interfaces::msg::OperationalStatus& msg);

  void finish() override;
};
