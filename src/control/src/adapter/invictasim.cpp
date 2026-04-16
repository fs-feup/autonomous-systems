#include "adapter/invictasim.hpp"

InvictaSimAdapter::InvictaSimAdapter(const ControlParameters& params)
    : ControlNode(params),
      control_pub_(
          create_publisher<custom_interfaces::msg::ControlCommand>("/control/command", 10)) {
  go_signal_ = true;

  if (this->params_.using_simulated_slam_) {
    vehicle_pose_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/invictasim/pose", 10,
        std::bind(&InvictaSimAdapter::pose_callback, this, std::placeholders::_1));
  }

  if (this->params_.using_simulated_velocities_) {
    vehicle_velocity_sub_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/invictasim/velocity", 10,
        std::bind(&InvictaSimAdapter::velocity_callback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(this->get_logger(), "Invictasim adapter created");
}

void InvictaSimAdapter::pose_callback(const custom_interfaces::msg::Pose& msg) {
  this->vehicle_pose_callback(msg);
}

void InvictaSimAdapter::velocity_callback(const custom_interfaces::msg::Velocities& msg) {
  this->vehicle_state_callback(msg);
}

void InvictaSimAdapter::publish_command(common_lib::structures::ControlCommand cmd) {
  custom_interfaces::msg::ControlCommand message;
  message.throttle_rr = cmd.throttle_rr;
  message.throttle_rl = cmd.throttle_rl;
  message.throttle_fr = cmd.throttle_fr;
  message.throttle_fl = cmd.throttle_fl;
  message.steering = cmd.steering_angle;

  this->control_pub_->publish(message);
}
