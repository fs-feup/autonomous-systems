#include "adapter_planning/pacsim.hpp"

#include "planning/planning.hpp"

PacSimAdapter::PacSimAdapter(Planning* planning) : Adapter(planning) { this->init(); }

void PacSimAdapter::init() {
  pacsim_ebs_server = this->node->create_client<std_srvs::srv::Empty>("pacsim/finish_signal");

  this->front_steering_sub = this->node->create_subscription<pacsim::msg::StampedScalar>(
      "/ros_can/state", 10,
      std::bind(&PacSimAdapter::front_steering_callback, this, std::placeholders::_1));

  this->rear_steering_sub = this->node->create_subscription<pacsim::msg::StampedScalar>(
      "/ros_can/state", 10,
      std::bind(&PacSimAdapter::rear_steering_callback, this, std::placeholders::_1));

  this->pose_sub = this->node->create_subscription<sensor_msgs::msg::JointState>(
      "/tf", 10, std::bind(&PacSimAdapter::publish_pose, this, std::placeholders::_1));
}

void PacSimAdapter::front_steering_callback(pacsim::msg::StampedScalar& msg) {
  this->provisional_front_steering = msg.value;
}

void PacSimAdapter::rear_steering_callback(pacsim::msg::StampedScalar& msg) {
  this->provisional_rear_steering = msg.value;
}

void PacSimAdapter::publish_pose(/*tipo de mensagem do pacsim para posição*/) {
  custom_interfaces::msg::VehicleState pose;
  pose.theta = (this->provisional_front_steering + this->provisional_rear_steering) / 2;
  pose.position.x = 0;  // provisional
  pose.position.y = 0;  // provisional
  this->node->vehicle_localization_callback(pose);
}