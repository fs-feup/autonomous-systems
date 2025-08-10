#include "node/node.hpp"

VENode::VENode(const VEParameters& parameters)
    : Node("velocity_estimation"), _parameters_(parameters) {
  this->_velocity_estimator_ = estimators_map_.at(parameters._estimation_method_)(parameters);
  this->_velocities_pub_ = this->create_publisher<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", 10);
}

void VENode::publish_velocities() const {
  common_lib::structures::Velocities state = this->_velocity_estimator_->get_velocities();
  custom_interfaces::msg::Velocities velocities_msg;
  velocities_msg.header.stamp = state.timestamp_;
  velocities_msg.velocity_x = state.velocity_x;
  velocities_msg.velocity_y = state.velocity_y;
  velocities_msg.angular_velocity = state.rotational_velocity;
  velocities_msg.covariance[0] = state.velocity_x_noise_;
  velocities_msg.covariance[4] = state.velocity_y_noise_;
  velocities_msg.covariance[8] = state.rotational_velocity_noise_;
  this->_velocities_pub_->publish(velocities_msg);
}