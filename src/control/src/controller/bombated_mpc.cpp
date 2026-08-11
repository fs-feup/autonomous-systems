#include "controller/bombated_mpc.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>

#define PATHPOINT_SIZE 4 // x, y, v, orientation
#define MIN_PATH_SIZE 5 // number of points in the path horizon

MPC::MPC(const ControlParameters& params) : Controller(params) {
  RCLCPP_INFO(rclcpp::get_logger("bombated_mpc"), "Initializing Bombated MPC Controller");
  this->solver_ = solver_map.at("bombated_mpc_acados")(params);
  this->local_pather_ = local_pather_map.at("alpha_beta")(params);
  this->path_data.pathpoint_array.resize((this->params_->mpc_prediction_horizon_steps_ + 1) * PATHPOINT_SIZE);
}

void MPC::set_path_in_solver() {

  custom_interfaces::msg::PathPointArray resampled_path;

  local_path_resampled_with_spline(this->latest_path_, this->solver_state_, this->local_pather_, this->params_->mpc_prediction_horizon_steps_, this->params_->mpc_prediction_horizon_seconds_, resampled_path);

  if (resampled_path.pathpoint_array.size() != this->params_->mpc_prediction_horizon_steps_ + 1) {
    RCLCPP_ERROR(rclcpp::get_logger("bombated_mpc"), "Resampled path has less points than the MPC horizon. Resampled points: %zu, required: %u. This can lead to unexpected behavior.", resampled_path.pathpoint_array.size(), this->params_->mpc_prediction_horizon_steps_ + 1);
    return;
  }
  this->path_data = resampled_path;
  this->solver_->set_path(resampled_path);
}

void MPC::path_callback(const custom_interfaces::msg::PathPointArray& new_msg) {
  if (new_msg.pathpoint_array.size() < MIN_PATH_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger("bombated_mpc"), "Received path has less than %d points, will be discarded. Received %zu points.", MIN_PATH_SIZE, new_msg.pathpoint_array.size());
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
  const double speed_threshold = 0.4;

  // Only ever stop because the PLANNER asked for a stop. The previous version
  // looked at the locally resampled reference, which goes quiet whenever the
  // horizon degenerates - and because a stopped car regenerates the very same
  // degenerate reference on the next cycle, that decision latched permanently.
  // The car would simply sit there with a zero command forever.
  if (std::fabs(this->solver_state_.velocity_x) > speed_threshold) {
    return false;
  }
  if (!this->_path_received_ || this->latest_path_.pathpoint_array.empty()) {
    return false;
  }

  // Look at the global path the planner actually published, near the car.
  const double x = this->solver_state_.x;
  const double y = this->solver_state_.y;
  size_t closest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < this->latest_path_.pathpoint_array.size(); ++i) {
    const auto& p = this->latest_path_.pathpoint_array[i];
    const double d = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
    if (d < best) { best = d; closest = i; }
  }

  // Stop only if the planner wants ~zero speed over the stretch ahead of us.
  constexpr size_t kLookahead = 10;
  double ahead = 0.0;
  size_t counted = 0;
  for (size_t k = 0; k < kLookahead; ++k) {
    const size_t idx = closest + k;
    if (idx >= this->latest_path_.pathpoint_array.size()) break;
    ahead += this->latest_path_.pathpoint_array[idx].v;
    ++counted;
  }
  if (counted == 0) return false;
  return (ahead / static_cast<double>(counted)) <= speed_threshold;
}

common_lib::structures::ControlCommand MPC::get_control_command() {
  if (!this->_path_received_) {
    return common_lib::structures::ControlCommand(); // Return zero command if path not received yet
  }

  this->set_path_in_solver();

  if (this->stopping_the_car()) {
    // Coast to a stop, but HOLD the steering. Snapping it to zero mid-corner
    // straightens the car into whatever it was turning away from.
    common_lib::structures::ControlCommand stop_command;
    stop_command.steering_angle = this->last_commanded_steering_;
    return stop_command;
  }

  this->solver_->set_state(this->solver_state_);
  int solver_status = 0;
  common_lib::structures::ControlCommand command = this->solver_->solve(&solver_status);
  this->last_commanded_steering_ = command.steering_angle;

  // Only assemble the (expensive) horizon dumps when a failure is actually going
  // to be reported. Building them on every 25 ms cycle wastes time in the
  // control loop, and printing them on every cycle of a sustained failure
  // floods the log to the point of masking the cause.
  if (solver_status < ACADOS_SUCCESS && should_report_failure()) {
    double x = this->solver_state_.x;
    double y = this->solver_state_.y;
    this->current_state = "State: x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", orientation: " + std::to_string(this->solver_state_.orientation) + ", vx: " + std::to_string(this->solver_state_.velocity_x) + ", vy: " + std::to_string(this->solver_state_.velocity_y) + ", yaw_rate: " + std::to_string(this->solver_state_.yaw_rate);
    this->computed_command = "Computed control command: throttle_rl=" + std::to_string(command.throttle_rl) + ", throttle_rr=" + std::to_string(command.throttle_rr) + ", steering_angle=" + std::to_string(command.steering_angle);

    std::vector<custom_interfaces::msg::VehicleStateVector> full_horizon = this->solver_->get_full_horizon();
    this->solver_state_over_horizon = "Full State over horizon: \n ";
    for (size_t i = 0; i < full_horizon.size(); ++i) {
      const auto& state = full_horizon[i];
      char buffer[256];
      snprintf(buffer, sizeof(buffer), "Stage %zu: x=%.3f, y=%.3f, theta=%.3f, v_x=%.3f, v_y=%.3f, yr=%.3f, a_x=%.3f, a_y=%.3f, steering=%.3f, fl=%.3f, fr=%.3f, rl=%.3f, rr=%.3f \n",
               i, state.x, state.y, state.orientation, state.velocity_x, state.velocity_y, state.yaw_rate, state.acceleration_x, state.acceleration_y, state.steering_angle, state.fl_rpm, state.fr_rpm, state.rl_rpm, state.rr_rpm);
      this->solver_state_over_horizon += buffer;
    }

    std::vector<common_lib::structures::ControlCommand> full_solution = this->solver_->get_full_solution();
    this->solver_command_over_horizon = "Commands over horizon: \n ";
    for (size_t i = 0; i + 1 < full_horizon.size() && i < full_solution.size(); ++i) {
      const auto& cmd = full_solution[i];
      char buffer[256];
      snprintf(buffer, sizeof(buffer), "Stage %zu: throttle_rl=%.3f, throttle_rr=%.3f, steering_angle=%.3f \n", i, cmd.throttle_rl, cmd.throttle_rr, cmd.steering_angle);
      this->solver_command_over_horizon += buffer;
    }

    this->print_debug_info();
  }

  return command;
}

bool MPC::should_report_failure() {
  const auto now = std::chrono::steady_clock::now();
  if (now - this->last_failure_report_ < std::chrono::seconds(2)) {
    return false;
  }
  this->last_failure_report_ = now;
  return true;
}

void MPC::print_debug_info() {
  std::cout << this->path_before_local << std::endl;
  std::cout << this->local_path_debug << std::endl;
  std::cout << this->current_state << std::endl;
  std::cout << this->computed_command << std::endl;
  std::cout << this->solver_state_over_horizon << std::endl;
  std::cout << this->solver_command_over_horizon << std::endl;
}