#include "solver/acados/acados.hpp"

#include <cmath>
#include <limits>

constexpr int kPathSize = 31;
constexpr int kPathPointSize = 4;
constexpr double kWeightEps = 0.5;

AcadosSolver::AcadosSolver(const ControlParameters& params) : SolverInterface(params), _execution_times_(std::make_shared<std::vector<double>>(9, 0.0)) {
    // 1. Create the capsule
    this->capsule_ = mpc_acados_create_capsule();
    
    // 2. Allocate solver memory
    int status = mpc_acados_create(this->capsule_);
    if (status != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "Failed to create Acados solver 'mpc', status: %d", status);
    }

    // 3. Cache internal pointers
    nlp_config_ = mpc_acados_get_nlp_config(this->capsule_);
    nlp_dims_ = mpc_acados_get_nlp_dims(this->capsule_);
    nlp_in_ = mpc_acados_get_nlp_in(this->capsule_);
    nlp_out_ = mpc_acados_get_nlp_out(this->capsule_);

    // 4. Initialize parameters per stage vector
    int N = this->control_params_->mpc_prediction_horizon_steps_;
    parameters_per_stage.resize((N+1)*4, 0.0); // Assuming 1 parameter
}

AcadosSolver::~AcadosSolver() {
    mpc_acados_free(this->capsule_);
    mpc_acados_free_capsule(this->capsule_);
}

void AcadosSolver::set_state(const std::vector<double>& x0) {
    if (x0.size() != 13) {
        RCLCPP_WARN(rclcpp::get_logger("AcadosSolver"), "State size mismatch: expected 13, got %zu", x0.size());
        return;
    }

    this->last_state_ = x0;
    this->has_state_ = true;

    std::vector<double> scaled_state = x0;

    scaled_state[9] /= this->control_params_->wheel_speeds_scale_mpc_;
    scaled_state[10] /= this->control_params_->wheel_speeds_scale_mpc_;
    scaled_state[11] /= this->control_params_->wheel_speeds_scale_mpc_;
    scaled_state[12] /= this->control_params_->wheel_speeds_scale_mpc_;

    // Set the initial state constraint (lbx and ubx) at stage 0
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", (void*)scaled_state.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", (void*)scaled_state.data());
}

void AcadosSolver::calculate_stage_parameters() {
    unsigned int path_point_count = this->last_path_.size() / kPathPointSize;

    // Get the horizon length and time step
    int N = this->control_params_->mpc_prediction_horizon_steps_;
    double time_step = this->control_params_->mpc_prediction_horizon_seconds_ / static_cast<double>(N);

    // Find the closest point on the line segment between first and second points
    double first_x = this->last_path_[0];
    double first_y = this->last_path_[1];
    double first_v = this->last_path_[2];
    double first_orientation = this->last_path_[3];
    double first_point_time = 0; // Can be the time of a path point or stage point
  
    double second_x = this->last_path_[kPathPointSize];
    double second_y = this->last_path_[kPathPointSize + 1];
    double second_v = this->last_path_[kPathPointSize + 2];
    double second_orientation = this->last_path_[kPathPointSize + 3];

    // Find the closest point on the line segment between first and second points
    double car_x = this->last_state_[0];
    double car_y = this->last_state_[1];
    double dx = second_x - first_x;
    double dy = second_y - first_y;
    double segment_length_sq = dx * dx + dy * dy;
    double t = 0.0;
    if (segment_length_sq > 1e-6) {
      t = ((car_x - first_x) * dx + (car_y - first_y) * dy) / segment_length_sq;
      t = std::max(0.0, std::min(1.0, t));
    }

    // First point is the closest
    double path_point_x = first_x + t * dx;
    double path_point_y = first_y + t * dy;
    double path_point_v = (1-t) * first_v + t * (second_v);
    double path_point_orientation = (1-t) * first_orientation + t * (second_orientation);
    double path_point_time = 0;

    double average_velocity = (path_point_v + second_v) / 2.0;
    double second_point_time = (1-t)*std::sqrt(segment_length_sq)/(average_velocity + 1e-6); // Assumes constant acceleration

    unsigned int segment_end_index = 1;
    // Iterate over all stages (0 to N) to set the time-varying parameters
    for (int i = 0; i <= N; ++i) {
      parameters_per_stage[i*4] = path_point_x;
      parameters_per_stage[i*4 + 1] = path_point_y;
      parameters_per_stage[i*4 + 2] = path_point_v;
      parameters_per_stage[i*4 + 3] = path_point_orientation;

      while (second_point_time - path_point_time < time_step && segment_end_index < path_point_count - 1) {
        segment_end_index++;

        first_x = second_x;
        first_y = second_y;
        first_v = second_v;
        first_orientation = second_orientation;
        first_point_time = second_point_time;

        second_x = this->last_path_[segment_end_index * kPathPointSize];
        second_y = this->last_path_[segment_end_index * kPathPointSize + 1];
        second_v = this->last_path_[segment_end_index * kPathPointSize + 2];
        second_orientation = this->last_path_[segment_end_index * kPathPointSize + 3];

        dx = second_x - first_x;
        dy = second_y - first_y;
        segment_length_sq = dx * dx + dy * dy;

        average_velocity = (first_v + second_v) / 2.0;
        second_point_time += std::sqrt(segment_length_sq)/(average_velocity + 1e-6);
      }

      double time_spent_on_segment = time_step - (first_point_time - path_point_time);
      double segment_delta_time = second_point_time - first_point_time;
      double segment_acceleration = (second_v - first_v) / (segment_delta_time + 1e-6);
      double average_velocity_in_segment = (first_v + (first_v + segment_acceleration * time_spent_on_segment)) / 2.0;
      double distance_traveled_in_segment = average_velocity_in_segment * time_spent_on_segment;

      t = distance_traveled_in_segment / (std::sqrt(segment_length_sq) + 1e-6);

      dx = second_x - first_x;
      dy = second_y - first_y;

      // First point is the closest
      path_point_x = first_x + t * dx;
      path_point_y = first_y + t * dy;
      path_point_v = first_v + segment_acceleration * time_spent_on_segment;
      path_point_orientation = (1-t) * first_orientation + t * (second_orientation);
      path_point_time += time_step;

      first_x = path_point_x;
      first_y = path_point_y;
      first_v = path_point_v;
      first_orientation = path_point_orientation;
      first_point_time = path_point_time;
      segment_length_sq = ( second_x - first_x) * (second_x - first_x) + (second_y - first_y) * (second_y - first_y);
    }
}

void AcadosSolver::initialize_solver_memory() {
  int N = this->control_params_->mpc_prediction_horizon_steps_;
  double u_zero[3] = {0.5, 0.5, 0.0}; // Baseline control guess
  double time_step = this->control_params_->mpc_prediction_horizon_seconds_ / static_cast<double>(N);

  double wheel_radius = 0.203;
  double wheelbase = 1.50;

  // Fill the state guess across the entire horizon (stages 0 to N)
  double next_yaw = 0;
  double next_v = 0;
  double yaw_rate = 0;
  double acceleration = 0;
  for (int i = 0; i <= N; ++i) {
    double x = this->parameters_per_stage[i*4];
    double y = this->parameters_per_stage[i*4 + 1];
    double v = this->parameters_per_stage[i*4 + 2];
    double yaw = this->parameters_per_stage[i*4 + 3];

    if (i != N) {
      next_yaw = this->parameters_per_stage[(i+1)*4 + 3];
      next_v = this->parameters_per_stage[(i+1)*4 + 2];
      yaw_rate = (next_yaw - yaw) / time_step;
      acceleration = (next_v - v) / time_step;
    }

    double steering = std::atan(wheelbase * yaw_rate / (v + 1e-6)); // Simple bicycle model for steering angle
    
    double wheel_speed_radians_scaled = (v / wheel_radius) / this->control_params_->wheel_speeds_scale_mpc_; // Convert linear velocity to angular velocity
    double state_guess[13] = {x, y, yaw, v, 0.0, yaw_rate, acceleration, 0.0, steering, wheel_speed_radians_scaled, wheel_speed_radians_scaled , wheel_speed_radians_scaled, wheel_speed_radians_scaled}; // [x, y, theta, v, ...] with some initial guess for velocities and other states
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", (void*)state_guess);
  }

  // Fill the control guess across the entire horizon (stages 0 to N-1)
  for (int i = 0; i < N; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", (void*)u_zero);
  }

  this->is_initialized_ = true;
}

void AcadosSolver::add_orientation(std::vector<double>& path_data) {
  unsigned int PATHPOINT_SIZE = 4; // Assuming each point has x, y, v, orientation
  // Add orientation for each point
  double prev_x = path_data[0];
  double prev_y = path_data[1];

  double current_x = path_data[PATHPOINT_SIZE];
  double current_y = path_data[PATHPOINT_SIZE + 1];

  double next_x = path_data[2 * PATHPOINT_SIZE];
  double next_y = path_data[2 * PATHPOINT_SIZE + 1];

  // Set orientation for the first point based on the first two points
  double dx = current_x - prev_x;
  double dy = current_y - prev_y;
  double orientation = std::atan2(dy, dx);
  path_data[3] = orientation;

  // Compute orientation for the rest of the points based on three consecutive points for better accuracy
  size_t i = PATHPOINT_SIZE;
  for (; i < path_data.size() - PATHPOINT_SIZE; i += PATHPOINT_SIZE) {
    next_x = path_data[i + PATHPOINT_SIZE];
    next_y = path_data[i + PATHPOINT_SIZE + 1];
    dx = next_x - prev_x;
    dy = next_y - prev_y;
    orientation = std::atan2(dy, dx);
    path_data[i + 3] = orientation;
    prev_x = current_x;
    prev_y = current_y;
    current_x = next_x;
    current_y = next_y;
  }

  // Set orientation for the last point based on the last two points
  dx = next_x - prev_x;
  dy = next_y - prev_y;
  orientation = std::atan2(dy, dx);
  path_data[path_data.size() - 1] = orientation;
}

void AcadosSolver::set_path_point_per_stage() {
  int N = this->control_params_->mpc_prediction_horizon_steps_;
  this->calculate_stage_parameters();
  //this->add_orientation(this->parameters_per_stage);
  this->stage_parameters_debug = "Stage parameters debug:  \n";
  for (int i = 0; i <= N; ++i) {
    double path_point_x = this->parameters_per_stage[i*4];
    double path_point_y = this->parameters_per_stage[i*4 + 1];
    double path_point_v = this->parameters_per_stage[i*4 + 2];
    double path_point_orientation = this->parameters_per_stage[i*4 + 3];
    this->stage_parameters_debug += "(" + std::to_string(path_point_x) + ", " + std::to_string(path_point_y) + ", " + std::to_string(path_point_v) + ", " + std::to_string(path_point_orientation) + ")\n";
    double point_for_stage[4] = {this->parameters_per_stage[i*4], this->parameters_per_stage[i*4 + 1], this->parameters_per_stage[i*4 + 2], this->parameters_per_stage[i*4 + 3]};
    mpc_acados_update_params(this->capsule_, i, point_for_stage, kPathPointSize);
  }
}

void AcadosSolver::update_mpc_stats() {
  // Create temporary variables to receive the raw values
  double t_tot, t_lin, t_sim, t_qp, t_reg;
  int sqp_iter;
  // Get the values from Acados (Times are in Seconds, Iterations is Int)
  ocp_nlp_solver *nlp_solver = mpc_acados_get_nlp_solver(this->capsule_);
  ocp_nlp_get(nlp_solver, "time_tot", &t_tot);
  ocp_nlp_get(nlp_solver, "time_lin", &t_lin);
  ocp_nlp_get(nlp_solver, "time_sim", &t_sim);
  ocp_nlp_get(nlp_solver, "time_qp",  &t_qp);
  ocp_nlp_get(nlp_solver, "time_reg", &t_reg);
  ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);
  // Convert times to milliseconds
  t_tot = t_tot * 1000.0;
  t_lin = t_lin * 1000.0;
  t_sim = t_sim * 1000.0;
  t_qp  = t_qp  * 1000.0;
  t_reg = t_reg * 1000.0;
  // Store in execution times vector for visualization
  (*_execution_times_)[0] = t_tot;
  (*_execution_times_)[1] = t_lin;
  (*_execution_times_)[2] = t_sim;
  (*_execution_times_)[3] = t_qp ;
  (*_execution_times_)[4] = t_reg;
  // Convert iteration count to double for easier visualization, even though it's an integer
  (*_execution_times_)[5] = (double)sqp_iter;
  // Update averages
  this->average_linearization_time_ = ((this->average_linearization_time_ * static_cast<double>(this->linearization_count_)) + t_lin) / static_cast<double>(this->linearization_count_ + 1);
  this->linearization_count_++;
  this->average_qp_time_ = ((this->average_qp_time_ * this->qp_count_) + t_qp) / static_cast<double>(this->qp_count_ + 1);   
  this->qp_count_++;
  this->average_regularization_time_ = ((this->average_regularization_time_ * this->regularization_count_) + t_reg) / static_cast<double>(this->regularization_count_ + 1);
  this->regularization_count_++;
  (*_execution_times_)[6] = average_linearization_time_;
  (*_execution_times_)[7] = average_qp_time_;
  (*_execution_times_)[8] = average_regularization_time_;
}

void AcadosSolver::set_path(const std::vector<double>& x_path) {
  this->last_path_ = x_path;
  this->has_path_ = true;
}

common_lib::structures::ControlCommand AcadosSolver::solve(int* solver_status) {
  common_lib::structures::ControlCommand command;
  if (!(this->has_state_ && this->has_path_)) {
    return command;
  }

  this->set_path_point_per_stage();

  double first_x = this->last_path_[0];
  double first_y = this->last_path_[1];
  double first_v = this->last_path_[2];
  double first_orientation = this->last_path_[3];
  first_x -= this->last_state_[0];
  first_y -= this->last_state_[1];
  first_orientation -= this->last_state_[2];
  first_v -= this->last_state_[3];
  //DEBUG PRINT
  if (std::fabs(first_x) > 0.01 || std::fabs(first_y) > 0.01 || std::fabs(first_v) > 0.01 || std::fabs(first_orientation) > 0.01) {
    RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "ERROR: first point doens't match state x:%.2f, y:%.2f, v:%.2f, orientation:%.2f", this->last_state_[0], this->last_state_[1], this->last_state_[2], this->last_state_[3]);
  }


  if (!this->is_initialized_) {
    this->initialize_solver_memory();
  }

  int status = mpc_acados_solve(this->capsule_);
  if (status != ACADOS_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "Acados solver failed with status %d", status);
    if (this->sanity_check_output()) {
      *solver_status = 1; // Positive to indicate benign failure (e.g. infeasibility)
    } else {
      print_debug_info(); // If solver failed, print debug info
      *solver_status = -1; // Negative to indicate malign failure
    }
  }

  this->update_mpc_stats();
  // Extracting 3 controls
  std::vector<common_lib::structures::ControlCommand> full_solution = this->get_full_solution();

  // Compensate for solver delay
  double total_solver_time_ms = this->_execution_times_->at(0);
  // Add an extra 5 ms as estimate for state estimation delay
  double total_delay_ms = total_solver_time_ms + 5.0;
  double total_delay_s = total_delay_ms / 1000.0;

  double time_step = this->control_params_->mpc_prediction_horizon_seconds_ / static_cast<double>(this->control_params_->mpc_prediction_horizon_steps_);
  unsigned int steps_ahead = static_cast<unsigned int>(std::floor(total_delay_s / time_step));
  if (steps_ahead >= full_solution.size() - 1) {
    RCLCPP_WARN(rclcpp::get_logger("AcadosSolver"), "Total delay of %.2f ms exceeds prediction horizon, using last available control", total_delay_ms);
    return full_solution.back();
  }

  common_lib::structures::ControlCommand command_zero = full_solution[steps_ahead];
  common_lib::structures::ControlCommand command_one = full_solution[steps_ahead + 1];

  // Linear interpolation of control commands
  double alpha = (total_delay_s - steps_ahead * time_step) / time_step;

  // DEBUG STRING
  this->total_delay_debug = "Total delay: " + std::to_string(total_delay_ms) + " ms, steps ahead: " + std::to_string(steps_ahead) + ", interpolation alpha: " + std::to_string(alpha);

  command.throttle_rl = (1 - alpha) * command_zero.throttle_rl + alpha * command_one.throttle_rl;
  command.throttle_rr = (1 - alpha) * command_zero.throttle_rr + alpha * command_one.throttle_rr;
  command.steering_angle = (1 - alpha) * command_zero.steering_angle + alpha * command_one.steering_angle;
  return command;
}

std::vector<common_lib::structures::ControlCommand> AcadosSolver::get_full_solution() {
  int N = nlp_dims_->N;
  std::vector<common_lib::structures::ControlCommand> full_u;
  full_u.reserve(N);
  
  for (int i = 0; i < N; ++i) {
    double solved_controls[3]; 
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", (void*)solved_controls);

    common_lib::structures::ControlCommand cmd;
    cmd.throttle_rl= solved_controls[0]; 
    cmd.throttle_rr= solved_controls[1];
    cmd.steering_angle = solved_controls[2];
    full_u.push_back(cmd);
  }
  return full_u;
}

std::vector<custom_interfaces::msg::VehicleStateVector> AcadosSolver::get_full_horizon() {
  int N = nlp_dims_->N;
  constexpr int kStateSize = 13;
  std::vector<custom_interfaces::msg::VehicleStateVector> full_horizon;
  full_horizon.reserve(N + 1);
  for (int i = 0; i <= N; ++i) {
    double solved_state[kStateSize] = {0.0};
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", (void*)solved_state);
    custom_interfaces::msg::VehicleStateVector state_vector;
    state_vector.x = solved_state[0];
    state_vector.y = solved_state[1];
    state_vector.orientation = solved_state[2];
    state_vector.velocity_x = solved_state[3];
    state_vector.velocity_y = solved_state[4];
    state_vector.yaw_rate = solved_state[5];
    state_vector.acceleration_x = solved_state[6];
    state_vector.acceleration_y = solved_state[7];
    state_vector.steering_angle = solved_state[8];
    state_vector.fl_rpm = solved_state[9];
    state_vector.fr_rpm = solved_state[10];
    state_vector.rl_rpm = solved_state[11];
    state_vector.rr_rpm = solved_state[12];
    full_horizon.emplace_back(state_vector);
  }
  return full_horizon;
}

void AcadosSolver::publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  if (publisher_map.find("/acados/execution_times") == publisher_map.end()) {
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/acados/execution_times", 10);
    publisher_map["/acados/execution_times"] = publisher;
  }

  auto publisher = std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>(publisher_map["/acados/execution_times"]);
  std_msgs::msg::Float64MultiArray msg;
  msg.data = *this->_execution_times_;
  publisher->publish(msg);
}

void AcadosSolver::print_debug_info() {
  std::cout << this->stage_parameters_debug << std::endl;
  std::cout << this->total_delay_debug << std::endl;
}

bool AcadosSolver::sanity_check_output() {
  // TODO: Implement actual checks
  return true;
}