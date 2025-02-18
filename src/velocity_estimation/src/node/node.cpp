#include "node/node.hpp"

VENode::VENode(const VEParameters& parameters)
    : Node("velocity_estimation"), _parameters_(parameters) {
  this->_velocity_estimator_ = create_estimator(parameters);
  this->_velocities_pub_ = this->create_publisher<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", 10);
}

void VENode::publish_velocities() const {
  auto state = this->_velocity_estimator_->get_velocities();
  custom_interfaces::msg::Velocities velocities_msg;
  velocities_msg.header.stamp = rclcpp::Clock().now();
  velocities_msg.velocity_x = state.velocity_x;
  velocities_msg.velocity_y = state.velocity_y;
  velocities_msg.angular_velocity = state.rotational_velocity;
  this->_velocities_pub_->publish(velocities_msg);
}