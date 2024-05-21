#include "adapter_control/pac_sim.hpp"

#include "node_/node_control.hpp"

PacSimAdapter::PacSimAdapter(Control *control)
    : Adapter(control),
      steering_pub(node->create_publisher<pacsim::msg::Wheels>("/pacsim/steering_setpoint", 10)),
      acceleration_pub(
          node->create_publisher<pacsim::msg::StampedScalar>("/pacsim/torques_max", 10)) {
  // No topic for pacsim, just set the go_signal to true
  node->go_signal = true;
}

void PacSimAdapter::publish_cmd(double acceleration, double steering) {
  auto steering_msg = pacsim::msg::Wheels();
  auto acceleration_msg = pacsim::msg::StampedScalar();

  // TODO: Convert values if necessary then fill the messages
  //  CODE HERE

  this->steering_pub->publish(steering_msg);
  this->acceleration_pub->publish(acceleration_msg);
}
