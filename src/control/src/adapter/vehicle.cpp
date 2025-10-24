#include "adapter/vehicle.hpp"

VehicleAdapter::VehicleAdapter(const ControlParameters& params)
    : ControlNode(params),
      go_sub_(create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&VehicleAdapter::go_signal_callback, this, std::placeholders::_1))),
      control_pub_(
          create_publisher<custom_interfaces::msg::ControlCommand>("/control/command", 10)) {
  RCLCPP_INFO(this->get_logger(), "Vehicle adapter created");
}

void VehicleAdapter::publish_command(common_lib::structures::ControlCommand cmd) {
  auto message = custom_interfaces::msg::ControlCommand();

  message.throttle_rr = cmd.throttle_rr;
  message.throttle_rl = cmd.throttle_rl;
  message.throttle_fr = cmd.throttle_fr;
  message.throttle_fl = cmd.throttle_fl;
  message.steering = cmd.steering_angle;

  this->control_pub_->publish(message);
}

void VehicleAdapter::go_signal_callback(const custom_interfaces::msg::OperationalStatus msg) {
  go_signal_ = msg.go_signal;
  if (!(msg.as_mission == common_lib::competition_logic::Mission::TRACKDRIVE) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::AUTOCROSS) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::SKIDPAD) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::ACCELERATION) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::EBS_TEST)) {
    go_signal_ = false;
  }
}