#include "node_/node_control.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "adapter_control/map.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This creates a subclass of Node and uses std::bind()
// to define which function gets executed at each time

// function that publishes data whenever a new path is obtained
void Control::publish_torque(custom_interfaces::msg::ConeArray path) {
  auto torque = std_msgs::msg::String();

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", torque.data.c_str());

  result->publish(torque);  // publish torque
}

void Control::velocity_estimation_callback(std_msgs::msg::String velocity) {}

Control::Control() : Node("control") {
  // get velocity data from state estimation
  current_velcoity = this->create_subscription<std_msgs::msg::String>(
      "velocity_estimation", 10,
      std::bind(&Control::velocity_estimation_callback, this, std::placeholders::_1));

  // get path (and ideal velocity associated) form planning
  path_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "local_planning", 10, std::bind(&Control::publish_torque, this, std::placeholders::_1));

  // creates publisher that should yield torque/acceleration/...
  result = this->create_publisher<std_msgs::msg::String>("torque_topic", 10);

  // Adapter to communicate with the car
  this->adapter = adapter_map[mode](this);
}