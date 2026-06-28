#include "lateral_controller/mpczinho.hpp"

#define PATHPOINT_SIZE 4 // x, y, v, orientation
#define MIN_PATH_SIZE 5 // number of points in the path horizon

MPCzinho::MPCzinho(const ControlParameters& params) : LateralController(params) {
  RCLCPP_INFO(rclcpp::get_logger("mpczinho"), "Initializing MPCzinho Controller");
  this->solver_ = solver_map.at("mpczinho_acados")(params);
  this->local_pather_ = local_pather_map.at("interpolator")(params);
}

void MPCzinho::vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& msg) {
  this->solver_state_.velocity_x = msg.velocity_x;
  this->solver_state_.velocity_y = msg.velocity_y;
  this->solver_state_.yaw_rate = msg.yaw_rate;
  this->solver_state_.acceleration_x = msg.acceleration_x;
  this->solver_state_.acceleration_y = msg.acceleration_y;
  this->solver_state_.steering_angle = msg.steering_angle;
  this->solver_state_.fl_rpm = msg.fl_rpm;
  this->solver_state_.fr_rpm = msg.fr_rpm;
  this->solver_state_.rl_rpm = msg.rl_rpm;
  this->solver_state_.rr_rpm = msg.rr_rpm;
}

void MPCzinho::vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) {
  this->solver_state_.x = msg.x;
  this->solver_state_.y = msg.y;
  this->solver_state_.orientation = msg.theta;
}

void MPCzinho::path_callback(const custom_interfaces::msg::PathPointArray& new_msg) {
  if (new_msg.pathpoint_array.size() < MIN_PATH_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger("mpczinho"), "Received path has less than %d points, will be discarded. Received %zu points.", MIN_PATH_SIZE, new_msg.pathpoint_array.size());
    return;
  }

  this->_path_received_ = true;
  this->latest_path_ = new_msg;
}

void MPCzinho::set_path_in_solver() {
  custom_interfaces::msg::PathPointArray resampled_path;

  local_path_resampled_with_spline(this->latest_path_, this->solver_state_, this->local_pather_, this->params_->mpc_prediction_horizon_steps_, this->params_->mpc_prediction_horizon_seconds_, resampled_path);

  if (resampled_path.pathpoint_array.size() != this->params_->mpc_prediction_horizon_steps_ + 1) {
    RCLCPP_ERROR(rclcpp::get_logger("mpczinho"), "Resampled path has less points than the MPC horizon. Resampled points: %zu, required: %u. This can lead to unexpected behavior.", resampled_path.pathpoint_array.size(), this->params_->mpc_prediction_horizon_steps_ + 1);
    return;
  }

  this->solver_->set_path(resampled_path);
}

double MPCzinho::get_steering_command() {
  if (!this->_path_received_) {
    return 0.0; // Return zero command if path not received yet
  }

  this->set_path_in_solver();

  this->solver_->set_state(this->solver_state_);
  int solver_status = 0;
  common_lib::structures::ControlCommand command = this->solver_->solve(&solver_status);

  return command.steering_angle;
}

void MPCzinho::publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  this->solver_->publish_solver_data(node, publisher_map);
}