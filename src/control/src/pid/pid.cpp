#include "pid/pid.hpp"

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Construct a new PID object
 *
 * @param kp Proporcional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param anti_windup Anti-windup constant
 * @param tau Derivative low pass filter time constant
 * @param t Sampling time
 * @param lim_min Minimum output value
 * @param lim_max Maximum output value
 */
PID::PID(double kp, double ki, double kd, double tau, double t, double lim_min, double lim_max,
         double anti_windup)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      anti_windup_(anti_windup),
      tau_(tau),
      t_(t),
      lim_min_(lim_min),
      lim_max_(lim_max) {}

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
  if (error > 4){
    error = 4;
  } else if (error < -2) {
    error = -2;
  }
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