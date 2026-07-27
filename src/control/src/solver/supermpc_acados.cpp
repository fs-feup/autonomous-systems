#include "solver/supermpc_acados/supermpc_acados.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

constexpr int path_point_size = 4;
// SuperMPC passes two extra per-stage parameters beyond the path point: the
// ambition speed and the ceiling speed, both derived from the planner's v_ref.
constexpr int kNumParams = 6;

// State layout of the generated supermpc model. Must stay in sync with
// supermpc_acados.py.
namespace {
constexpr int kStateSize = 16;
constexpr int kInputSize = 2;
constexpr int kIdxSteer = 8;
constexpr int kIdxWFl = 9;
constexpr int kIdxThrottleCmd = 13;
constexpr int kIdxSteerCmd = 14;
constexpr int kIdxThrottleApplied = 15;
constexpr double kMaxSteeringAngle = 0.335;
// Speed above which full regen is allowed; below it the request tapers linearly
// so nothing deep is ever queued into the inverter's transport delay.
constexpr double kRegenTaperSpeed = 4.0;
// Number of consecutive failed solves after which the warm start is discarded
// and rebuilt from the reference, rather than iterating on a poisoned guess.
constexpr int kMaxConsecutiveFailures = 5;
// After this many consecutive failures the solver is destroyed and rebuilt.
constexpr int kMaxFailuresBeforeReset = 25;
}  // namespace

SuperMpcAcadosSolver::SuperMpcAcadosSolver(const ControlParameters& params) : SolverInterface(params), _execution_times_(std::make_shared<std::vector<double>>(9, 0.0)) {
    // 1. Create the capsule
    this->capsule_ = supermpc_acados_create_capsule();
    
    // 2. Allocate solver memory
    int status = supermpc_acados_create(this->capsule_);
    if (status != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SuperMpcAcadosSolver"), "Failed to create Acados solver 'supermpc', status: %d", status);
    }

    // 3. Cache internal pointers
    nlp_config_ = supermpc_acados_get_nlp_config(this->capsule_);
    nlp_dims_ = supermpc_acados_get_nlp_dims(this->capsule_);
    nlp_in_ = supermpc_acados_get_nlp_in(this->capsule_);
    nlp_out_ = supermpc_acados_get_nlp_out(this->capsule_);

    // 4. Initialize parameters per stage vector
    int N = this->control_params_->supermpc_prediction_horizon_steps_;
    parameters_per_stage.resize((N+1)*4, 0.0); // Assuming 1 parameter

    // 5. Override the generated cost weights and envelope with the config
    apply_cost_weights();
    apply_envelope();
}

void SuperMpcAcadosSolver::reset_solver() {
  supermpc_acados_free(this->capsule_);
  supermpc_acados_free_capsule(this->capsule_);
  this->capsule_ = supermpc_acados_create_capsule();
  if (supermpc_acados_create(this->capsule_) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SuperMpcAcadosSolver"), "Failed to rebuild the acados solver");
    return;
  }
  nlp_config_ = supermpc_acados_get_nlp_config(this->capsule_);
  nlp_dims_ = supermpc_acados_get_nlp_dims(this->capsule_);
  nlp_in_ = supermpc_acados_get_nlp_in(this->capsule_);
  nlp_out_ = supermpc_acados_get_nlp_out(this->capsule_);
  apply_cost_weights();
  apply_envelope();
  this->is_initialized_ = false;
}

void SuperMpcAcadosSolver::apply_cost_weights() {
  // Push the configured cost weights into the solver at construction time so
  // they can be tuned from YAML without regenerating the acados C code. The
  // weights baked in by the generator remain the fallback when the config omits
  // them (or gives the wrong number of entries).
  const int N = nlp_dims_->N;
  auto set_stage_weights = [this](const std::vector<double>& weights, int stage,
                                  const char* label) {
    if (weights.empty()) return false;
    int dims_out[2] = {0, 0};
    ocp_nlp_cost_dims_get_from_attr(nlp_config_, nlp_dims_, nlp_out_, stage, "W", dims_out);
    const int ny = dims_out[0];
    if (static_cast<int>(weights.size()) != ny) {
      RCLCPP_ERROR(rclcpp::get_logger("SuperMpcAcadosSolver"),
                   "%s has %zu entries but stage %d expects %d; keeping generated weights",
                   label, weights.size(), stage, ny);
      return false;
    }
    std::vector<double> W(static_cast<size_t>(ny) * ny, 0.0);
    for (int i = 0; i < ny; ++i) W[i * ny + i] = weights[i];
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "W", W.data());
    return true;
  };

  bool ok = true;
  for (int stage = 0; stage < N; ++stage) {
    ok &= set_stage_weights(this->control_params_->supermpc_cost_weights_, stage, "supermpc_cost_weights");
  }
  ok &= set_stage_weights(this->control_params_->supermpc_terminal_cost_weights_, N,
                          "supermpc_terminal_cost_weights");
  if (ok && !this->control_params_->supermpc_cost_weights_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("SuperMpcAcadosSolver"), "Applied cost weights from config");
  }
}


SuperMpcAcadosSolver::~SuperMpcAcadosSolver() {
  supermpc_acados_free(this->capsule_);
  supermpc_acados_free_capsule(this->capsule_);
}

void SuperMpcAcadosSolver::set_state(const custom_interfaces::msg::VehicleStateVector& state) {
  this->latest_state_ = state;
  this->has_state_ = true;

  std::vector<double> scaled_state(kStateSize, 0.0);

  scaled_state[0] = state.x;
  scaled_state[1] = state.y;
  scaled_state[2] = state.orientation;
  scaled_state[3] = state.velocity_x;
  scaled_state[4] = state.velocity_y;
  scaled_state[5] = state.yaw_rate;
  scaled_state[6] = state.acceleration_x;
  scaled_state[7] = state.acceleration_y;
  scaled_state[kIdxSteer] = state.steering_angle;
  scaled_state[kIdxWFl + 0] = state.fl_rpm / this->control_params_->wheel_speeds_scale_mpc_;
  scaled_state[kIdxWFl + 1] = state.fr_rpm / this->control_params_->wheel_speeds_scale_mpc_;
  scaled_state[kIdxWFl + 2] = state.rl_rpm / this->control_params_->wheel_speeds_scale_mpc_;
  scaled_state[kIdxWFl + 3] = state.rr_rpm / this->control_params_->wheel_speeds_scale_mpc_;

  // The command states are internal to the controller: the plant does not
  // report them back, so they are seeded from what was actually sent last
  // cycle. Without this the rate-integrated commands would restart from zero
  // every solve and the car would stutter.
  scaled_state[kIdxThrottleCmd] = this->last_throttle_command_;
  scaled_state[kIdxSteerCmd] = this->last_steering_command_;
  // The inverter lag state is not observable either; the actual steering angle
  // is, so only the throttle path needs an internal estimate.
  scaled_state[kIdxThrottleApplied] = this->applied_throttle_estimate_;

  // Set the initial state constraint (lbx and ubx) at stage 0
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", (void*)scaled_state.data());
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", (void*)scaled_state.data());
}

void SuperMpcAcadosSolver::initialize_solver_memory() {
  int N = this->control_params_->supermpc_prediction_horizon_steps_;
  double u_zero[kInputSize] = {0.0, 0.0}; // Rates start at zero

  constexpr double kWheelRadius = 0.20574;

  // Cold start from the measured state held constant over the horizon.
  //
  // The previous guess was built by differentiating the reference orientation
  // to get a yaw rate and inverting a kinematic bicycle for the steering angle.
  // That reference is noisy at the point spacing used here, so the guess had
  // the steering slamming between +-0.335 on consecutive stages - a trajectory
  // that satisfies neither the steering actuator's 0.112 s lag nor the rate
  // bounds. SQP_RTI takes a single step from whatever it is given, so an
  // aggressively infeasible guess makes the very first QP hit ACADOS_MINSTEP,
  // and because RTI then warm-starts from that iterate it never recovers.
  //
  // A constant trajectory at the current state is always inside every bound and
  // is close to dynamically feasible at low speed, which is where the solver is
  // started.
  const double scale = this->control_params_->wheel_speeds_scale_mpc_;
  const double wheel_speed_scaled = (this->latest_state_.velocity_x / kWheelRadius) / scale;

  double state_guess[kStateSize] = {
      this->latest_state_.x,
      this->latest_state_.y,
      this->latest_state_.orientation,
      this->latest_state_.velocity_x,
      this->latest_state_.velocity_y,
      this->latest_state_.yaw_rate,
      this->latest_state_.acceleration_x,
      this->latest_state_.acceleration_y,
      this->latest_state_.steering_angle,
      wheel_speed_scaled, wheel_speed_scaled, wheel_speed_scaled, wheel_speed_scaled,
      this->last_throttle_command_,
      this->last_steering_command_,
      this->applied_throttle_estimate_};

  for (int i = 0; i <= N; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", (void*)state_guess);
  }

  // Fill the control guess across the entire horizon (stages 0 to N-1)
  for (int i = 0; i < N; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", (void*)u_zero);
  }

  this->is_initialized_ = true;
}

void SuperMpcAcadosSolver::apply_envelope() {
  // Rear slip angle envelope, set at runtime so it can be tuned from YAML.
  const double limit = this->control_params_->supermpc_max_rear_slip_;
  if (!(limit > 0.0)) return;
  const double lower[2] = {-limit, -limit};
  const double upper[2] = {limit, limit};
  const int N = nlp_dims_->N;
  for (int stage = 1; stage < N; ++stage) {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, "lh",
                                  (void*)lower);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, "uh",
                                  (void*)upper);
  }
  RCLCPP_INFO(rclcpp::get_logger("SuperMpcAcadosSolver"),
              "Rear slip angle envelope set to +-%.3f rad", limit);
}

void SuperMpcAcadosSolver::set_path_point_per_stage() {
  int N = this->control_params_->supermpc_prediction_horizon_steps_;
  this->stage_parameters_debug = "Stage parameters debug:  \n";
  for (int i = 0; i <= N; ++i) {
    double path_point_x = this->parameters_per_stage[i*path_point_size];
    double path_point_y = this->parameters_per_stage[i*path_point_size + 1];
    double path_point_v = this->parameters_per_stage[i*path_point_size + 2];
    double path_point_orientation = this->parameters_per_stage[i*path_point_size + 3];
    this->stage_parameters_debug += "(" + std::to_string(path_point_x) + ", " + std::to_string(path_point_y) + ", " + std::to_string(path_point_v) + ", " + std::to_string(path_point_orientation) + ")\n";
    // The planner's speed becomes two thresholds rather than a target: a mildly
    // optimistic one the cost gently pulls towards, and a hard-ish ceiling above
    // which a heavy one-sided penalty applies. Between them the controller is
    // free, and its own dynamics decide the speed.
    const double v_stretch = path_point_v * this->control_params_->supermpc_speed_stretch_;
    const double v_cap = path_point_v * this->control_params_->supermpc_speed_cap_;
    double point_for_stage[kNumParams] = {path_point_x, path_point_y, path_point_v,
                                          path_point_orientation, v_stretch, v_cap};
    supermpc_acados_update_params(this->capsule_, i, point_for_stage, kNumParams);
  }
}

void SuperMpcAcadosSolver::update_mpc_stats() {
  // Create temporary variables to receive the raw values
  double t_tot, t_lin, t_sim, t_qp, t_reg;
  int sqp_iter;
  // Get the values from Acados (Times are in Seconds, Iterations is Int)
  ocp_nlp_solver *nlp_solver = supermpc_acados_get_nlp_solver(this->capsule_);
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

void SuperMpcAcadosSolver::set_path(const custom_interfaces::msg::PathPointArray& path) {
  if (path.pathpoint_array.size() != static_cast<size_t>(this->control_params_->supermpc_prediction_horizon_steps_ + 1)) {
    RCLCPP_ERROR(rclcpp::get_logger("SuperMpcAcadosSolver"), "Received path with %zu points, but expected %d points based on MPC horizon. Ignoring path update.", path.pathpoint_array.size(), this->control_params_->supermpc_prediction_horizon_steps_ + 1);
    return;
  }

  for (size_t i = 0; i < path.pathpoint_array.size(); ++i) {
    const auto& point = path.pathpoint_array[i];
    this->parameters_per_stage[i*path_point_size] = point.x;
    this->parameters_per_stage[i*path_point_size + 1] = point.y;
    this->parameters_per_stage[i*path_point_size + 2] = point.v;
    this->parameters_per_stage[i*path_point_size + 3] = point.orientation;
  }

  this->has_path_ = true;
}

common_lib::structures::ControlCommand SuperMpcAcadosSolver::solve(int* solver_status) {
  common_lib::structures::ControlCommand command;
  if (!(this->has_state_ && this->has_path_)) {
    return command;
  }

  this->set_path_point_per_stage();

  double first_x = this->parameters_per_stage[0];
  double first_y = this->parameters_per_stage[1];
  double first_v = this->parameters_per_stage[2];
  double first_orientation = this->parameters_per_stage[3];
  first_x -= this->latest_state_.x;
  first_y -= this->latest_state_.y;
  first_orientation -= this->latest_state_.orientation;
  first_v -= this->latest_state_.velocity_x;
  //DEBUG PRINT
  if (std::fabs(first_x) > 0.01 || std::fabs(first_y) > 0.01 || std::fabs(first_v) > 0.01 || std::fabs(first_orientation) > 0.01) {
    RCLCPP_ERROR(rclcpp::get_logger("SuperMpcAcadosSolver"), "ERROR: first point doens't match state x:%.2f, y:%.2f, v:%.2f, orientation:%.2f", this->latest_state_.x, this->latest_state_.y, this->latest_state_.velocity_x, this->latest_state_.orientation);
  }


  if (!this->is_initialized_) {
    this->initialize_solver_memory();
  }

  int status = supermpc_acados_solve(this->capsule_);
  this->update_mpc_stats();

  if (status != ACADOS_SUCCESS) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("SuperMpcAcadosSolver"), this->throttle_clock_, 1000,
                         "Acados solver failed with status %d", status);
    // SQP_RTI warm-starts from the previous iterate, so a bad iterate is sticky.
    // Rebuild the guess from the reference after a short run of failures.
    if (++this->consecutive_failures_ >= kMaxConsecutiveFailures) {
      this->initialize_solver_memory();
    }
    // Re-seeding the guess is not always enough: acados carries internal QP
    // state (duals, slacks, Hessian) across calls, and once that is poisoned
    // every later solve fails too. The controller then sits in this fallback
    // forever - throttle 0, steering frozen - and the car coasts off the
    // track. Tear the solver down and rebuild it rather than latch.
    if (this->consecutive_failures_ >= kMaxFailuresBeforeReset) {
      RCLCPP_WARN(rclcpp::get_logger("SuperMpcAcadosSolver"),
                  "%d consecutive solver failures - rebuilding the solver",
                  this->consecutive_failures_);
      this->reset_solver();
      this->consecutive_failures_ = 0;
    }
    if (this->sanity_check_output()) {
      *solver_status = 1; // Positive to indicate benign failure (e.g. infeasibility)
    } else {
      *solver_status = -1; // Negative to indicate malign failure
      // A diverged solution must never reach the car: a spurious negative
      // throttle drives it backwards and a spurious steering angle spins it.
      // Hold the last good steering angle and coast instead.
      command.throttle_rl = 0.0;
      command.throttle_rr = 0.0;
      command.steering_angle = this->last_steering_command_;
      this->last_throttle_command_ = 0.0;
      this->applied_throttle_estimate_ +=
          (0.0 - this->applied_throttle_estimate_) * kInverterBlend;
      return command;
    }
  }

  this->consecutive_failures_ = 0;

  std::vector<common_lib::structures::ControlCommand> full_solution = this->get_full_solution();
  if (full_solution.size() < 2) {
    return command;
  }

  // Compensate for solver delay
  double total_solver_time_ms = this->_execution_times_->at(0);
  // Add an extra 5 ms as estimate for state estimation delay
  double total_delay_ms = total_solver_time_ms + 5.0;
  double total_delay_s = total_delay_ms / 1000.0;

  double time_step = this->control_params_->supermpc_prediction_horizon_seconds_ / static_cast<double>(this->control_params_->supermpc_prediction_horizon_steps_);
  unsigned int steps_ahead = static_cast<unsigned int>(std::floor(total_delay_s / time_step));
  if (steps_ahead >= full_solution.size() - 1) {
    RCLCPP_WARN(rclcpp::get_logger("SuperMpcAcadosSolver"), "Total delay of %.2f ms exceeds prediction horizon, using last available control", total_delay_ms);
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

  // Never publish a command outside the actuator's physical range. The command
  // states are only SOFT-bounded in the OCP (a hard bound there can make the QP
  // infeasible), and the plant does not clamp its steering STATE - it clamps
  // only the wheel angles - so an out-of-range command is fed straight back in
  // as the measured steering angle on the next cycle and the pair runs away
  // together.
  command.throttle_rl = std::clamp(command.throttle_rl, -1.0, 1.0);
  command.throttle_rr = std::clamp(command.throttle_rr, -1.0, 1.0);
  command.steering_angle =
      std::clamp(command.steering_angle, -kMaxSteeringAngle, kMaxSteeringAngle);

  // The plant has NO BRAKES in autonomous mode: FSFEUP02 only builds brake
  // torques when control_mode_ == "manual", so a negative throttle goes straight
  // to the inverter as negative MOTOR torque, which below a standstill drives
  // the car backwards.
  //
  // A hard cutoff at some speed is NOT enough, because DelayedInverter queues
  // the request for |throttle| * 500 ms. Full regen asked for at 4 m/s is still
  // being applied half a second later, when the car has already stopped, and it
  // then pushes the car backwards. The limit therefore tapers with speed: never
  // queue more regen than the car will still have speed to absorb on arrival.
  const double regen_limit =
      std::clamp(this->latest_state_.velocity_x / kRegenTaperSpeed, 0.0, 1.0);
  command.throttle_rl = std::max(command.throttle_rl, -regen_limit);
  command.throttle_rr = std::max(command.throttle_rr, -regen_limit);

  // Remember what was actually sent: these are the seeds for the command states
  // on the next solve, and the fallback value if the solver fails.
  this->last_throttle_command_ = command.throttle_rl;
  this->last_steering_command_ = command.steering_angle;
  this->applied_throttle_estimate_ +=
      (command.throttle_rl - this->applied_throttle_estimate_) * kInverterBlend;
  return command;
}

std::vector<common_lib::structures::ControlCommand> SuperMpcAcadosSolver::get_full_solution() {
  int N = nlp_dims_->N;
  std::vector<common_lib::structures::ControlCommand> full_u;
  full_u.reserve(N);

  // The commands are integrated STATES in this formulation (the inputs are
  // their rates), so they are read from the state trajectory at stages 1..N.
  for (int i = 1; i <= N; ++i) {
    double solved_state[kStateSize] = {0.0};
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", (void*)solved_state);

    common_lib::structures::ControlCommand cmd;
    // The simulator averages the two rear throttles and the Salisbury
    // differential splits the axle torque, so there is no torque vectoring to
    // command: both wheels get the same request.
    cmd.throttle_rl = solved_state[kIdxThrottleCmd];
    cmd.throttle_rr = solved_state[kIdxThrottleCmd];
    cmd.steering_angle = solved_state[kIdxSteerCmd];
    full_u.push_back(cmd);
  }
  return full_u;
}

std::vector<custom_interfaces::msg::VehicleStateVector> SuperMpcAcadosSolver::get_full_horizon() {
  int N = nlp_dims_->N;
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
    state_vector.steering_angle = solved_state[kIdxSteer];
    state_vector.fl_rpm = solved_state[kIdxWFl + 0];
    state_vector.fr_rpm = solved_state[kIdxWFl + 1];
    state_vector.rl_rpm = solved_state[kIdxWFl + 2];
    state_vector.rr_rpm = solved_state[kIdxWFl + 3];
    full_horizon.emplace_back(state_vector);
  }
  return full_horizon;
}

void SuperMpcAcadosSolver::publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  if (publisher_map.find("/acados/execution_times") == publisher_map.end()) {
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/acados/execution_times", 10);
    publisher_map["/acados/execution_times"] = publisher;
  }

  auto publisher = std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>(publisher_map["/acados/execution_times"]);
  std_msgs::msg::Float64MultiArray msg;
  msg.data = *this->_execution_times_;
  publisher->publish(msg);

  this->publish_interpolated_path(node, publisher_map);
  this->publish_received_state(node, publisher_map);
}

void SuperMpcAcadosSolver::publish_received_state(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  if (!this->has_state_) return;

  const std::string topic = "/supermpc/received_state";
  if (publisher_map.find(topic) == publisher_map.end()) {
    publisher_map[topic] = node->create_publisher<custom_interfaces::msg::VehicleStateVector>(topic, 10);
  }

  auto state_publisher = std::static_pointer_cast<rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>>(publisher_map[topic]);
  state_publisher->publish(this->latest_state_);
}

void SuperMpcAcadosSolver::publish_interpolated_path(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  if (!this->has_path_) return;

  const std::string topic = "/supermpc/interpolated_path";
  if (publisher_map.find(topic) == publisher_map.end()) {
    publisher_map[topic] = node->create_publisher<visualization_msgs::msg::Marker>(topic, 10);
  }

  // Rebuild the interpolated trajectory received by the solver for visualization
  int N = this->control_params_->supermpc_prediction_horizon_steps_;
  std::vector<common_lib::structures::PathPoint> interpolated_path;
  interpolated_path.reserve(N + 1);
  for (int i = 0; i <= N; ++i) {
    interpolated_path.emplace_back(this->parameters_per_stage[i * path_point_size],
                                   this->parameters_per_stage[i * path_point_size + 1],
                                   this->parameters_per_stage[i * path_point_size + 3],
                                   this->parameters_per_stage[i * path_point_size + 2]);
  }

  auto path_publisher = std::static_pointer_cast<rclcpp::Publisher<visualization_msgs::msg::Marker>>(publisher_map[topic]);
  path_publisher->publish(common_lib::communication::line_marker_from_structure_array(
      interpolated_path, "supermpc_interpolated_path", "map", 0, "blue"));
}

void SuperMpcAcadosSolver::print_debug_info() {
  std::cout << this->stage_parameters_debug << std::endl;
  std::cout << this->total_delay_debug << std::endl;
}

bool SuperMpcAcadosSolver::sanity_check_output() {
  // TODO: Implement actual checks
  return false;
}