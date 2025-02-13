#include "node/node.hpp"

VENode::VENode(const VEParameters& parameters)
    : Node("velocity_estimation"), _parameters_(parameters) {
  this->_velocity_estimator_ = create_estimator(parameters);
  this->_velocities_pub_ = this->create_publisher<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", 10);
}