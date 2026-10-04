#pragma once

#include "common_lib/competition_logic/mission_logic.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "ros_node/ros_node.hpp"

/**
 * @brief Adapter for interfacing with the invictasim simulator.
 */
class InvictasimAdapter : public ControlNode {
private:
  /**
   * @brief Subscription for the go signal from the operational status, which activates
   * the controller when true.
   *
   */
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr go_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr velocities_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr pose_sub_;
  /**
   * @brief Subscription for the vehicle state, used to obtain the longitudinal and lateral
   * acceleration in the car's frame, the steering angle and the individual wheel speeds, which
   * the velocities topic does not provide.
   */
  rclcpp::Subscription<custom_interfaces::msg::VehicleStateVector>::SharedPtr vehicle_status_sub_;

  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_pub_;

public:
  explicit InvictasimAdapter(const ControlParameters &params);
  void publish_command(common_lib::structures::ControlCommand cmd) override;
  void go_signal_callback(const custom_interfaces::msg::OperationalStatus msg);
  void velocities_callback(const custom_interfaces::msg::Velocities &msg);
  void pose_callback(const custom_interfaces::msg::Pose &msg);
  void vehicle_status_callback(const custom_interfaces::msg::VehicleStateVector &msg);
};
