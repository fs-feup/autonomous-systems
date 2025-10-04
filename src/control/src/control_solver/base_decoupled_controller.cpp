#include "control_solver/base_decoupled_controller.hpp"

void BaseDecoupledController::path_callback(const custom_interfaces::msg::PathPointArray& msg) {
  lateral_controller_->path_callback(msg);
  longitudinal_controller_->path_callback(msg);
}

void BaseDecoupledController::vehicle_state_callback(const custom_interfaces::msg::Velocities::SharedPtr msg) {
  lateral_controller_->vehicle_state_callback(msg);
  longitudinal_controller_->vehicle_state_callback(msg);
}

void BaseDecoupledController::vehicle_pose_callback(const custom_interfaces::msg::Pose& vehicle_state_msg) {
  lateral_controller_->vehicle_pose_callback(vehicle_state_msg);
  longitudinal_controller_->vehicle_pose_callback(vehicle_state_msg);
}

common_lib::structures::ControlCommand BaseDecoupledController::get_control_command() {
  common_lib::structures::ControlCommand command = this->longitudinal_controller_->get_throttle_command();
  command.steering_angle = this->lateral_controller_->get_steering_command();
  return command;
}