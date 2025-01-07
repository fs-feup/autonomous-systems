#include "node/bridge.hpp"

ControlBridgeNode::ControlBridgeNode() : Node("control_bridge_node") {
  // Subscribe to the input control topics
  control_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/as_msgs/controls", 10,
      std::bind(&ControlBridgeNode::controlCallback, this, std::placeholders::_1));

  // Publish to the output control topics for the simulator
  acceleration_pub_ = this->create_publisher<pacsim::msg::Wheels>("/pacsim/torques_max", 10);
  steering_pub_ =
      this->create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10);
}

void ControlBridgeNode::controlCallback(const custom_interfaces::msg::ControlCommand msg) {
  auto acceleration_msg = pacsim::msg::Wheels();
  auto steering_msg = pacsim::msg::StampedScalar();

  // Convert and fill the acceleration message
  acceleration_msg.fl = acceleration_msg.fr = acceleration_msg.rl = acceleration_msg.rr =
      msg.throttle;
  steering_msg.value = msg.steering;

  acceleration_pub_->publish(acceleration_msg);
  steering_pub_->publish(steering_msg);
}