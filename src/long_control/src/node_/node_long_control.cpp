#include "node_/node_long_control.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

void LongitudinalControl::publish_torque(
    custom_interfaces::msg::ConeArray path) {
  auto torque = std_msgs::msg::String();

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", torque.data.c_str());
  result->publish(torque);
}
void LongitudinalControl::velocity_estimation_callback(std_msgs::msg::String velocity) {}

LongitudinalControl::LongitudinalControl() : Node("node_long_control") {
  current_velcoity = this->create_subscription<std_msgs::msg::String>(
      "velocity_estimation", 10,
      std::bind(&LongitudinalControl::velocity_estimation_callback, this,
                std::placeholders::_1));  // get data from state estimation
  path_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "local_planning", 10,
      std::bind(&LongitudinalControl::publish_torque, this, std::placeholders::_1));

  result = this->create_publisher<std_msgs::msg::String>(
      "torque_topic", 10);  // changed the name of topic from "topic" to "torque_topic"
}