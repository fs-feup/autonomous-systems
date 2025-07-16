#include "pid/pid.hpp"

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Construct a new PID object
 *
 * @param params ControlParameters struct containing PID parameters
 */
PID::PID(const ControlParameters& params)
    : kp_(params.pid_kp_),
      ki_(params.pid_ki_),
      kd_(params.pid_kd_),
      anti_windup_(params.pid_anti_windup_),
      tau_(params.pid_tau_),
      t_(params.pid_t_),
      lim_min_(params.pid_lim_min_),
      lim_max_(params.pid_lim_max_) {}

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

  /*
   * Proportional term
   */
  this->calculate_proportional_term(error);

  /*
   * Integral term
   */
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
  return setpoint - measurement;
}

void PID::calculate_proportional_term(double error) { this->proportional_ = this->kp_ * error; }

void PID::calculate_integral_term(double error) {
  this->integrator_ = this->integrator_ + 0.5f * this->ki_ * this->t_ * (error + this->prev_error_);
}

void PID::anti_wind_up() {
  double curr_output = this->proportional_ + this->integrator_ + this->differentiator_;

  if (curr_output > this->lim_max_ || curr_output < this->lim_min_) {
    this->integrator_ = this->integrator_ * this->anti_windup_;
  }
}

void PID::calculate_derivative_term(double measurement) {
  this->differentiator_ = (-2.0f * this->kd_ * (measurement - this->prev_measurement_) +
                           (2.0f * this->tau_ - this->t_) * this->differentiator_) /
                          (2.0f * this->tau_ + this->t_);
}

void PID::compute_output() {
  this->out_ = this->proportional_ + this->integrator_ + this->differentiator_;

  if (this->out_ > this->lim_max_) {
    this->out_ = this->lim_max_;
  } else if (this->out_ < this->lim_min_) {
    this->out_ = this->lim_min_;
  }
}

PID::PID() = default;