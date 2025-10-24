#pragma once

#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "ros_node/ros_node.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"

/**
 * @brief Adapter for interfacing with the real vehicle hardware.
 *
 * @details Works on a publish-subscribe model. Publishes commands to the vehicle
 * in the form of ControlCommand messages and subscribes to OperationalStatus messages
 * to receive a go signal.
 */
class VehicleAdapter : public ControlNode {
private:
  /**
   * @brief Subscription for the go signal from the operational status, which activates
   * the controller when true.
   *
   */
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr go_sub_;

  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_pub_;

public:
  explicit VehicleAdapter(const ControlParameters &params);
  void publish_command(common_lib::structures::ControlCommand cmd) override;
  void go_signal_callback(const custom_interfaces::msg::OperationalStatus msg);
};
