#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "config/parameters.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "utils/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "controller/map.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


/**
 * @class ControlNode
 * @brief Class responsible for the ROS2 communication of the control module
 *
 * This class inherits from rclcpp::Node, subscribing to current state (velocity), pose
 * and path topics, and publishing a control command.
 */
class ControlNode : public rclcpp::Node {
protected:
  bool go_signal_{false};
  ControlParameters params_;

  /**
   * @brief Called when a new vehicle pose is received
   * @param msg The received pose message
   */
  void vehicle_pose_callback(const custom_interfaces::msg::Pose &msg);

  /**
   * @brief Called when a new path is received
   * @param msg The received path message
   */
  void path_callback(const custom_interfaces::msg::PathPointArray &msg);

  /**
   * @brief Called when a new velocity is received
   * @param msg The received velocity message
   */
  void vehicle_state_callback(const custom_interfaces::msg::Velocities &msg);
private:
  // Vector of execution times for different parts of the control loop 
  // Currently just the first element is used, which is the total execution time
  std::shared_ptr<std::vector<double>> _execution_times_;

  // Controller (lateral + longitudinal)
  std::shared_ptr<Controller> controller_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr execution_time_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Subscriptions
  rclcpp::Subscription<custom_interfaces::msg::PathPointArray>::SharedPtr path_point_array_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vehicle_pose_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr velocity_sub_;

  /**
   * @brief Function that publishes control commands on timer ticks
   */
  void control_timer_callback();

  /**
   * @brief Adapters override this function to publish control commands in
   * their environment
   * @param cmd Control command to be published
   */
  virtual void publish_command(common_lib::structures::ControlCommand cmd) = 0;

public:
  explicit ControlNode(const ControlParameters &params);
};