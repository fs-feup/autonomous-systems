#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include "custom_interfaces/msg/operational_status.hpp"
#include "ros_node/slam_node.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class VehicleAdapter : public SLAMNode {
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr
      _operational_status_subscription_;  ///< Subscriber for operational status
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _lidar_odometry_subscription_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      _finished_client_;  ///< Client for finished signal

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
      _tf_static_broadcaster_;  ///< Static transform broadcaster

public:
  /**
   * @brief Constructor of the vehicle adapter node
   */
  VehicleAdapter(const SLAMParameters& params);

  /**
   * @brief LiDAR odometry subscription callback
   *
   * @param msg Pose estimated by the LiDAR odometry
   */
  void _lidar_odometry_subscription_callback(const nav_msgs::msg::Odometry& msg);

  /**
   * @brief Sends the finished signal to the vehicle control
   */
  void finish() override;
};