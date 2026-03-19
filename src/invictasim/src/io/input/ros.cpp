#include "io/input/ros.hpp"

RosInputAdapter::RosInputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : Node("invictasim_input", rclcpp::NodeOptions().use_global_arguments(false)),
      InvictaSimInputAdapter(simulator) {
  control_command_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/control/command", 10, [this](const custom_interfaces::msg::ControlCommand::SharedPtr msg) {
        common_lib::structures::Wheels throttle;
        throttle.front_left = msg->throttle_fl;
        throttle.front_right = msg->throttle_fr;
        throttle.rear_left = msg->throttle_rl;
        throttle.rear_right = msg->throttle_rr;
        simulator_->set_input(throttle, msg->steering);
      });
}
