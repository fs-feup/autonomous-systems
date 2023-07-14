#include "loc_map/lm_ekf_node.hpp"

EKFNode::EKFNode(ExtendedKalmanFilter* ekf) : Node("ekf_node"), _ekf(ekf) {
  this->start();  // TODO(marhcouto): check if this is the best place to start the timer
}

void EKFNode::start() {
  this->_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&EKFNode::_timer_callback, this));
}

void EKFNode::_timer_callback() {
  this->_ekf->prediction_step();
  this->_ekf->correction_step();
  this->_ekf->update();
  VehicleState* vehicle_state = this->_ekf->get_vehicle_state();
  RCLCPP_INFO(this->get_logger(), "EFK Updated state: x:%lf  y:%lf  theta:%lf",
              vehicle_state->pose.position.x, vehicle_state->pose.position.y,
              vehicle_state->pose.orientation);
  for (auto& cone : this->_ekf->get_map()->map) {
    RCLCPP_INFO(this->get_logger(), "(%lf, %lf)\t%s", cone.first.x, cone.first.y,
                colors::color_names[cone.second]);
  }
  RCLCPP_INFO(this->get_logger(), "--------------------------------------");
}