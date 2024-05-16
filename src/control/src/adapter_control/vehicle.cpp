#include "adapter_control/vehicle.hpp"

VehicleAdapter::VehicleAdapter(Control *control)
    : Adapter(control),
      go_sub(node->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&VehicleAdapter::finish, this, std::placeholders::_1))),
      control_pub_(node->create_publisher<custom_interfaces::msg::ControlCommand>(
          "/car/control_command", 10)) {}

void VehicleAdapter::publish_cmd(float acceleration, float steering) {
  auto message = custom_interfaces::msg::ControlCommand();

  // Convert values to range according to the units sent by AS
  message.throttle = acceleration;
  message.steering = steering;

  this->control_pub_->publish(message);
}

void VehicleAdapter::finish() {
  // Service as well, to fill in eventually
}