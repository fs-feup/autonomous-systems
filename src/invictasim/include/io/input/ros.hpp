#pragma once

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/perception_output.hpp"
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
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr
      slam_map_sub_;  ///< Subscription for SLAM map cones.
  rclcpp::Subscription<custom_interfaces::msg::PerceptionOutput>::SharedPtr
      perception_sub_;  ///< Subscription for external perception cones.
  rclcpp::Subscription<custom_interfaces::msg::PathPointArray>::SharedPtr
      path_sub_;  ///< Subscription for external path points.
};
