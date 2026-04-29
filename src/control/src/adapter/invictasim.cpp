#include "adapter/invictasim.hpp"

InvictasimAdapter::InvictasimAdapter(const ControlParameters &params)
    : ControlNode(params),
      go_sub_(create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/invictasim/operational_status", 10,
          std::bind(&InvictasimAdapter::go_signal_callback, this, std::placeholders::_1))),
      control_pub_(
          create_publisher<custom_interfaces::msg::ControlCommand>("/control/command", 10)) {
  if (this->params_.using_simulated_slam_) {
    pose_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/invictasim/state_estimation/pose", 1,
        std::bind(&InvictasimAdapter::pose_callback, this, std::placeholders::_1));
  }

  if (this->params_.using_simulated_velocities_) {
    velocities_sub_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/invictasim/state_estimation/velocities", 1,
        std::bind(&InvictasimAdapter::velocities_callback, this, std::placeholders::_1));
  }
  RCLCPP_INFO(this->get_logger(), "Invictasim adapter created");
}

void InvictasimAdapter::publish_command(common_lib::structures::ControlCommand cmd) {
  auto message = custom_interfaces::msg::ControlCommand();

  message.throttle_rr = cmd.throttle_rr;
  message.throttle_rl = cmd.throttle_rl;
  message.throttle_fr = cmd.throttle_fr;
  message.throttle_fl = cmd.throttle_fl;
  message.steering = cmd.steering_angle;

  this->control_pub_->publish(message);
}

void InvictasimAdapter::go_signal_callback(const custom_interfaces::msg::OperationalStatus msg) {
  go_signal_ = msg.go_signal;
  if (!(msg.as_mission == common_lib::competition_logic::Mission::TRACKDRIVE) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::AUTOCROSS) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::SKIDPAD) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::ACCELERATION) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::EBS_TEST)) {
    go_signal_ = false;
  }
}

void InvictasimAdapter::velocities_callback(const custom_interfaces::msg::Velocities &msg) {
  this->vehicle_state_callback(msg);
}

void InvictasimAdapter::pose_callback(const custom_interfaces::msg::Pose &msg) {
  this->vehicle_pose_callback(msg);
}