#pragma once

#include <mutex>

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
   */
  RosInputAdapter();

  /**
   * @brief Access method for the simulator class.
   * @return Current input.
   */
  InvictaSimInput get_current_input() const override;

private:
  /**
   * @brief Store the latest received input.
   * @param input New simulator input.
   */
  void set_current_input(const InvictaSimInput& input);

  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr
      control_command_sub_;         ///< Subscription for control commands.
  mutable std::mutex input_mutex_;  ///< Protects access to the current input.
  InvictaSimInput current_input_;   ///< Latest command received from ROS.
};
