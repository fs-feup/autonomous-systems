#include "io/input/ros.hpp"

RosInputAdapter::RosInputAdapter()
    : Node("invictasim_input", rclcpp::NodeOptions().use_global_arguments(false)) {
  control_command_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/control/command", 10, [this](const custom_interfaces::msg::ControlCommand::SharedPtr msg) {
        InvictaSimInput input;
        input.throttle.front_left = msg->throttle_fl;
        input.throttle.front_right = msg->throttle_fr;
        input.throttle.rear_left = msg->throttle_rl;
        input.throttle.rear_right = msg->throttle_rr;
        input.steering = msg->steering;
        set_current_input(input);
      });
}

InvictaSimInput RosInputAdapter::get_current_input() const {
  std::lock_guard<std::mutex> lock(input_mutex_);
  return current_input_;
}

void RosInputAdapter::set_current_input(const InvictaSimInput& input) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  current_input_ = input;
}
