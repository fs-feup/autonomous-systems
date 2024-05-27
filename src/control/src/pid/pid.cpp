#include "pid/pid.hpp"

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Construct a new PID object
 *
 * @param Kp Proporcional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param antiWindup Anti-windup constant
 * @param tau Derivative low pass filter time constant
 * @param T Sampling time
 * @param limMin Minimum output value
 * @param limMax Maximum output value
 */

PID::PID(double Kp, double Ki, double Kd, double tau, double T, double limMin, double limMax,
         double antiWindup)
    : Kp(Kp),
      Ki(Ki),
      Kd(Kd),
      antiWindup(antiWindup),
      tau(tau),
      T(T),
      limMin(limMin),
      limMax(limMax) {}

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
  this->prevError = error;
  this->prevMeasurement = measurement;

  /*
   * return output value
   */
  return this->out;
}

double PID::calculate_error(double setpoint, double measurement) const {
  return setpoint - measurement;
}

void PID::calculate_proportional_term(double error) { this->proportional = this->Kp * error; }

void PID::calculate_integral_term(double error) {
  this->integrator = this->integrator + 0.5f * this->Ki * this->T * (error + this->prevError);
}

void PID::anti_wind_up() {
  double curr_output = this->proportional + this->integrator + this->differentiator;

  if (curr_output > this->limMax || curr_output < this->limMin) {
    this->integrator = this->integrator * this->antiWindup;
  }
}

void PID::calculate_derivative_term(double measurement) {
  this->differentiator = (-2.0f * this->Kd * (measurement - this->prevMeasurement) +
                          (2.0f * this->tau - this->T) * this->differentiator) /
                         (2.0f * this->tau + this->T);
}

void PID::compute_output() {
  this->out = this->proportional + this->integrator + this->differentiator;

  if (this->out > this->limMax) {
    this->out = this->limMax;
  } else if (this->out < this->limMin) {
    this->out = this->limMin;
  }
}

PID::PID() = default;