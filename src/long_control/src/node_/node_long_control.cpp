#include "node_/node_long_control.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//This example creates a subclass of Node and uses std::bind()
//to define which function gets executed at each time


//function that publishes data whenever a new path is obtained
void LongitudinalControl::publish_torque(custom_interfaces::msg::ConeArray path) {
  auto torque = std_msgs::msg::String();

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", torque.data.c_str());
  //publish torque
  result->publish(torque);
}

//function to transfer the velocity data to a variable
void LongitudinalControl::velocity_estimation_callback(std_msgs::msg::String velocity) {}


//constructor
LongitudinalControl::LongitudinalControl() : Node("node_long_control") {
  // get velocity data from state estimation
  current_velcoity = this->create_subscription<std_msgs::msg::String>("velocity_estimation",
  10, std::bind(&LongitudinalControl::velocity_estimation_callback, this, std::placeholders::_1));

  // get path (and ideal velocity associated) form planning
  path_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>("local_planning",
  10, std::bind(&LongitudinalControl::publish_torque, this, std::placeholders::_1));

  //creates publisher that should yield torque/acceleration/...
  result = this->create_publisher<std_msgs::msg::String>("torque_topic", 10);
}