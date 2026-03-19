#pragma once

#include "custom_interfaces/msg/control_command.hpp"
#include "io/input/input_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief ROS-based simulator input adapter.
 */
class RosInputAdapter : public rclcpp::Node, public InvictaSimInputAdapter {
public:
  /**
   * @brief Construct a new RosInputAdapter.
   * @param simulator Simulator instance.
   */
  explicit RosInputAdapter(const std::shared_ptr<InvictaSim>& simulator);

private:
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr
      control_command_sub_;  ///< Subscription for control commands.
};
