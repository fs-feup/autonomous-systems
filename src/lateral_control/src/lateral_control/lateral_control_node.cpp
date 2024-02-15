#include "lateral_control/lateral_control_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @brief Publish Steering command when a new path is received
 *
 * @param path - the new path
 * @return void
 */
void LateralControl::publish_steeringcommand(custom_interfaces::msg::ConeArray path) {
  auto steering_command_msg = std_msgs::msg::String();

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", steering_command_msg.data.c_str());
  steering_command_pub->publish(steering_command_msg);
}

/**
 * @brief Publish the EBS Status
 *
 * @param tbd
 * @return void
 */
void LateralControl::publish_latctrl_ebs_status() {
  auto ebs_status_msg = std_msgs::msg::Bool();

  RCLCPP_INFO(this->get_logger(), "Lateral Control EBS Status: '%d'", ebs_status_msg.data);
  ebs_status_pub->publish(ebs_status_msg);
}

/**
 * @brief Class Constructor
 */
LateralControl::LateralControl() : Node("lateral_control_node") {
  ebs_status_pub = this->create_publisher<std_msgs::msg::Bool>("ebs_status_topic", 10);
  steering_command_pub =
      this->create_publisher<std_msgs::msg::String>("steering_command_topic", 10);

  path_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "local_planning", 10,
      std::bind(&LateralControl::publish_steeringcommand, this, std::placeholders::_1));

  // Adapter to communicate with the car
  // TODO: mode is set somewhere not hardcoded
  this->adapter = new Adapter("fsds", this);
}