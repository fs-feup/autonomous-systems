#pragma once

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "pacsim/msg/perception_detections.hpp"
#include "ros_node/slam_node.hpp"

class PacsimAdapter : public SLAMNode {
  rclcpp::Subscription<pacsim::msg::PerceptionDetections>::SharedPtr
      _perception_detections_subscription_;  ///< Subscriber for simulated perception detections

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      _velocities_subscription_;  ///< Subscriber for simulated velocities

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr
      _finished_client_;  ///< Client for finished signal

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client_;  // for mission logic

  /**
   * @brief Callback for simulated perception detections from pacsim
   *
   * @param msg Message containing the array of perceived detections
   */
  void _pacsim_perception_subscription_callback(const pacsim::msg::PerceptionDetections& msg);

  /**
   * @brief Callback for simulated velocities from pacsim
   *
   * @param msg Message containing the velocities of the vehicle
   */
  void _pacsim_velocities_subscription_callback(
      const geometry_msgs::msg::TwistWithCovarianceStamped& msg);

  /**
   * @brief Fetches the mission from the parameters.
   */
  void fetch_discipline();

public:
  /**
   * @brief Constructor of the pacsim adapter node
   */
  PacsimAdapter(const SLAMParameters& params);

  void finish() override;
};