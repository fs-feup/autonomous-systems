#include "longitudinal_controller/pid.hpp"

using namespace common_lib::structures;

/**
 * @brief Construct a new PID object
 *
 */
PID::PID(const ControlParameters& params) : LongitudinalController(params) {}

/**
 * @brief Calculate the output value
 *
 * @param setpoint
 * @param measurement
 * @return double
 */

double PID::update(double setpoint, double measurement) {
  /*
   * Error signal
   */
  double error = calculate_error(setpoint, measurement);

  this->calculate_proportional_term(error);

  this->calculate_integral_term(error);

  /*
   * Derivative term , derivative on measurement
   */
  this->calculate_derivative_term(measurement);

  /*
   * Anti-wind-up integrator
   */
  this->anti_wind_up();

  /*
   * Compute output and apply limits
   */
  this->compute_output();

  /*
   * Store error and measurement for the next iteration
   */
  this->prev_error_ = error;
  this->prev_measurement_ = measurement;

  /*
   * return output value
   */
  return this->out_;
}

double PID::calculate_error(double setpoint, double measurement) const {
  double error = setpoint - measurement;
  if (error > this->params_->pid_max_positive_error_) {
    error = this->params_->pid_max_positive_error_;
  } else if (error < this->params_->pid_max_negative_error_) {
    error = this->params_->pid_max_negative_error_;
  }
  return error;
}

void PID::calculate_proportional_term(double error) {
  this->proportional_ = this->params_->pid_kp_ * error;
}

void PID::calculate_integral_term(double error) {
  this->integrator_ = this->integrator_ + 0.5f * this->params_->pid_ki_ * this->params_->pid_t_ *
                                              (error + this->prev_error_);
}

void PID::anti_wind_up() {
  double curr_output = this->proportional_ + this->integrator_ + this->differentiator_;
  if (curr_output > this->params_->pid_lim_max_ || curr_output < this->params_->pid_lim_min_) {
    this->integrator_ = this->integrator_ * this->params_->pid_anti_windup_;
  }
}

void PID::calculate_derivative_term(double measurement) {
  this->differentiator_ =
      (-2.0f * this->params_->pid_kd_ * (measurement - this->prev_measurement_) +
       (2.0f * this->params_->pid_tau_ - this->params_->pid_t_) * this->differentiator_) /
      (2.0f * this->params_->pid_tau_ + this->params_->pid_t_);
}

void PID::compute_output() {
  this->out_ = this->proportional_ + this->integrator_ + this->differentiator_;

  if (this->out_ > this->params_->pid_lim_max_) {
    this->out_ = this->params_->pid_lim_max_;
  } else if (this->out_ < this->params_->pid_lim_min_) {
    this->out_ = this->params_->pid_lim_min_;
  }
}

void PID::path_callback(const custom_interfaces::msg::PathPointArray& msg)  {
  this->last_path_msg_ = msg.pathpoint_array;
  this->received_first_path_ = true;
}
void PID::vehicle_state_callback(const custom_interfaces::msg::Velocities& msg)  {
  this->last_velocity_msg_ = msg;
  this->absolute_velocity_ = std::sqrt(msg.velocity_x * msg.velocity_x + msg.velocity_y * msg.velocity_y);
  this->received_first_state_ = true;
}
void PID::vehicle_pose_callback(const custom_interfaces::msg::Pose& msg)  {
  this->last_pose_msg_ = msg;
  this->received_first_pose_ = true;
}

common_lib::structures::ControlCommand PID::get_throttle_command()  {
  common_lib::structures::ControlCommand command;
  if (!this->received_first_path_ || !this->received_first_state_ || !this->received_first_pose_) {
    return command;
  }

  custom_interfaces::msg::PathPoint car_position;
  car_position.x = this->last_pose_msg_.x;
  car_position.y = this->last_pose_msg_.y;
  car_position.v = this->absolute_velocity_;
  this->last_path_msg_.insert(this->last_path_msg_.begin(), car_position);

  Position vehicle_cog = Position(this->last_pose_msg_.x, this->last_pose_msg_.y);
  Position rear_axis = rear_axis_position(vehicle_cog, this->last_pose_msg_.theta,
      this->params_->car_parameters_.dist_cg_2_rear_axis);

  auto [closest_point, closest_point_id, closest_point_velocity] =
      get_closest_point(this->last_path_msg_, rear_axis);

  if (closest_point_id == -1) {
    return command;
  }

  command.throttle_rl = command.throttle_rr = update(closest_point_velocity, this->absolute_velocity_);
  return command;
}