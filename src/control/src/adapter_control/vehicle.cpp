#include "adapter_control/vehicle.hpp"

VehicleAdapter::VehicleAdapter(const ControlParameters& params)
    : Control(params),
      go_sub_(create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&VehicleAdapter::go_signal_callback, this, std::placeholders::_1))),
      control_pub_(
          create_publisher<custom_interfaces::msg::ControlCommand>("/car/control_command", 10)) {
  RCLCPP_INFO(this->get_logger(), "Vehicle adapter created");
}

void VehicleAdapter::publish_cmd(double acceleration, double steering) {
  auto message = custom_interfaces::msg::ControlCommand();

  // Convert values to range according to the units sent by AS
  message.throttle = acceleration;
  message.steering = steering;

  this->control_pub_->publish(message);
}

void VehicleAdapter::go_signal_callback(const custom_interfaces::msg::OperationalStatus msg) {
  // No need to do anything with the message, just set the go_signal to true
  go_signal_ = msg.go_signal;
}