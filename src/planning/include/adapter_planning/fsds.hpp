#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "custom_interfaces/msg/vehicle_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "planning/planning.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

//Probably useless

class FsdsAdapter : public Planning {
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fsds_position_subscription_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

public:
  explicit FsdsAdapter(const PlanningParameters& params);

  void mission_state_callback(const fs_msgs::msg::GoSignal msg);
  void set_mission_state();
  void pose_callback(const nav_msgs::msg::Odometry& msg);
  void finish() override;
};