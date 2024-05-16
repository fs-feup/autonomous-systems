#include "pid/pid.hpp"

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

PID::PID(float Kp, float Ki, float Kd, float tau, float T, float limMin, float limMax,
         float antiWindup)
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
 * @return float
 */

float PID::update(float setpoint, float measurement) {
  /*
   * Error signal
   */
  float error = calculate_error(setpoint, measurement);

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

float PID::calculate_error(float setpoint, float measurement) const {
  return setpoint - measurement;
}

void PID::calculate_proportional_term(float error) { this->proportional = this->Kp * error; }

void PID::calculate_integral_term(float error) {
  this->integrator = this->integrator + 0.5f * this->Ki * this->T * (error + this->prevError);
}

void PID::anti_wind_up() {
  float curr_output = this->proportional + this->integrator + this->differentiator;

  if (curr_output > this->limMax || curr_output < this->limMin) {
    this->integrator = this->integrator * this->antiWindup;
  }
}

void PID::calculate_derivative_term(float measurement) {
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