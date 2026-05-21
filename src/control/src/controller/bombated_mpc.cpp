#include "controller/bombated_mpc.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

#define PATHPOINT_SIZE 4 // x, y, v, orientation
#define MIN_PATH_SIZE 5 // number of points in the path horizon

MPC::MPC(const ControlParameters& params) : Controller(params) {
  RCLCPP_INFO(rclcpp::get_logger("bombated_mpc"), "Initializing Bombated MPC Controller");
  this->solver_ = solver_map.at("bombated_mpc_acados")(params);
  this->local_pather_ = local_pather_map.at("interpolator")(params);
  this->path_data.pathpoint_array.resize((this->params_->mpc_prediction_horizon_steps_ + 1) * PATHPOINT_SIZE);
}

void MPC::set_path_in_solver() {

  custom_interfaces::msg::PathPointArray resampled_path;

  local_path_resampled_with_spline(this->latest_path_, this->solver_state_, this->local_pather_, this->params_->mpc_prediction_horizon_steps_, this->params_->mpc_prediction_horizon_seconds_, resampled_path);

  if (resampled_path.pathpoint_array.size() != this->params_->mpc_prediction_horizon_steps_ + 1) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc"), "Resampled path has less points than the MPC horizon. Resampled points: %zu, required: %u. This can lead to unexpected behavior.", resampled_path.pathpoint_array.size(), this->params_->mpc_prediction_horizon_steps_ + 1);
    return;
  }
  this->path_data = resampled_path;
  this->solver_->set_path(resampled_path);
}

void MPC::path_callback(const custom_interfaces::msg::PathPointArray& new_msg) {
  if (new_msg.pathpoint_array.size() < MIN_PATH_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc"), "Received path has less than %d points, will be discarded. Received %zu points.", MIN_PATH_SIZE, new_msg.pathpoint_array.size());
    return;
  }

  this->_path_received_ = true;
  this->latest_path_ = new_msg;
}

void MPC::vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& msg) {
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

void MPC::vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) {
  this->solver_state_.x = msg.x;
  this->solver_state_.y = msg.y;
  this->solver_state_.orientation = msg.theta;
}

void MPC::publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  this->solver_->publish_solver_data(node, publisher_map);
}

bool MPC::stopping_the_car() {
  double speed_threshold = 0.4; // m/s

  // Check current speed
  if (std::fabs(this->solver_state_.velocity_x) > speed_threshold) {
    return false;
  }

  // Check average goal speed over the horizon
  double goal_speed_average = 0;
  for (size_t i = 0; i <= this->params_->mpc_prediction_horizon_steps_; ++i) {
    goal_speed_average += this->path_data.pathpoint_array[i].v;
  }
  goal_speed_average /= (this->params_->mpc_prediction_horizon_steps_ + 1);
  if (goal_speed_average > speed_threshold) {
    return false;
  }
  
  // Check if speed is tending towards 0: calculate average speed goal variation
  double speed_variation_average = 0;
  for (size_t i = 0; i < this->params_->mpc_prediction_horizon_steps_; ++i) {
    double current_goal_speed = this->path_data.pathpoint_array[i].v;
    double next_goal_speed = this->path_data.pathpoint_array[i + 1].v;
    speed_variation_average += (next_goal_speed - current_goal_speed);
  }
  speed_variation_average /= this->params_->mpc_prediction_horizon_steps_;
  double current_velocity = this->solver_state_.velocity_x;
  double allowed_negative_speed_due_to_noise = -0.1;
  if (speed_variation_average > 0 && current_velocity > allowed_negative_speed_due_to_noise) {
    return false;
  }

  return true;
}

common_lib::structures::ControlCommand MPC::get_control_command() {
  if (!this->_path_received_) {
    return common_lib::structures::ControlCommand(); // Return zero command if path not received yet
  }

  this->set_path_in_solver();

  if (this->stopping_the_car()) {
    common_lib::structures::ControlCommand stop_command; // all 0, maybe dangerous if the steering isn't at 0, TODO: reconsider
    return stop_command;
  }

  this->solver_->set_state(this->solver_state_);
  int solver_status = 0;
  common_lib::structures::ControlCommand command = this->solver_->solve(&solver_status);

  // DEBUG STRING
  double x = this->solver_state_.x;
  double y = this->solver_state_.y;
  this->current_state = "State: x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", orientation: " + std::to_string(this->solver_state_.orientation) + ", vx: " + std::to_string(this->solver_state_.velocity_x) + ", vy: " + std::to_string(this->solver_state_.velocity_y) + ", yaw_rate: " + std::to_string(this->solver_state_.yaw_rate);
  this->computed_command = "Computed control command: throttle_rl=" + std::to_string(command.throttle_rl) + ", throttle_rr=" + std::to_string(command.throttle_rr) + ", steering_angle=" + std::to_string(command.steering_angle);

  // DEBUG STRING
  std::vector<custom_interfaces::msg::VehicleStateVector> full_horizon = this->solver_->get_full_horizon();
  this->solver_state_over_horizon = "Full State over horizon: \n ";
  for (size_t i = 0; i < full_horizon.size(); ++i) {
    const auto& state = full_horizon[i];
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Stage %zu: x=%.3f, y=%.3f, theta=%.3f, v_x=%.3f, v_y=%.3f, yr=%.3f, a_x=%.3f, a_y=%.3f, steering=%.3f, fl=%.3f, fr=%.3f, rl=%.3f, rr=%.3f \n",
             i, state.x, state.y, state.orientation, state.velocity_x, state.velocity_y, state.yaw_rate, state.acceleration_x, state.acceleration_y, state.steering_angle, state.fl_rpm, state.fr_rpm, state.rl_rpm, state.rr_rpm);
    this->solver_state_over_horizon += buffer;
  }

  // DEBUG STRING
  std::vector<common_lib::structures::ControlCommand> full_solution = this->solver_->get_full_solution();
  this->solver_command_over_horizon = "Commands over horizon: \n ";
  for (size_t i = 0; i < full_horizon.size() - 1; ++i) { 
    const auto& cmd = full_solution[i];
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Stage %zu: throttle_rl=%.3f, throttle_rr=%.3f, steering_angle=%.3f \n", i, cmd.throttle_rl, cmd.throttle_rr, cmd.steering_angle);
    this->solver_command_over_horizon += buffer;
  }

  if (solver_status < ACADOS_SUCCESS) {
    this->print_debug_info();
  }

  return command;
}

void MPC::print_debug_info() {
  std::cout << this->path_before_local << std::endl;
  std::cout << this->local_path_debug << std::endl;
  std::cout << this->current_state << std::endl;
  std::cout << this->computed_command << std::endl;
  std::cout << this->solver_state_over_horizon << std::endl;
  std::cout << this->solver_command_over_horizon << std::endl;
}