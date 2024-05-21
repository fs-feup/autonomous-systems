#include "adapter_control/vehicle.hpp"

VehicleAdapter::VehicleAdapter(Control *control)
    : Adapter(control),
      go_sub(node->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&VehicleAdapter::go_signal_callback, this, std::placeholders::_1))),
      control_pub_(node->create_publisher<custom_interfaces::msg::ControlCommand>(
          "/car/control_command", 10)) {}

void VehicleAdapter::publish_cmd(double acceleration, double steering) {
  auto message = custom_interfaces::msg::ControlCommand();

  // Convert values to range according to the units sent by AS
  message.throttle = acceleration;
  message.steering = steering;

  this->control_pub_->publish(message);
}

void VehicleAdapter::go_signal_callback(const custom_interfaces::msg::OperationalStatus msg) {
  // No need to do anything with the message, just set the go_signal to true
  node->go_signal = msg.go_signal;
}