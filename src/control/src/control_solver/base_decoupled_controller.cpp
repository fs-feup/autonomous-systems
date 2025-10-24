#include "control_solver/base_decoupled_controller.hpp"

DecoupledController::DecoupledController(const ControlParameters& params) : ControlSolver(params),
 lateral_controller_(lateral_controller_map.at(params.lateral_controller_)(params)),
 longitudinal_controller_(longitudinal_controller_map.at(params.longitudinal_controller_)(params))  {}

void DecoupledController::path_callback(const custom_interfaces::msg::PathPointArray& msg) {
  lateral_controller_->path_callback(msg);
  longitudinal_controller_->path_callback(msg);
}

void DecoupledController::vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) {
  lateral_controller_->vehicle_state_callback(msg);
  longitudinal_controller_->vehicle_state_callback(msg);
}

void DecoupledController::vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) {
  lateral_controller_->vehicle_pose_callback(msg);
  longitudinal_controller_->vehicle_pose_callback(msg);
}

common_lib::structures::ControlCommand DecoupledController::get_control_command() {
  common_lib::structures::ControlCommand command = this->longitudinal_controller_->get_throttle_command();
  command.steering_angle = this->lateral_controller_->get_steering_command();
  return command;
}