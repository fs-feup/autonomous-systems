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

PID::PID(float Kp, float Ki, float Kd, float tau, float T, float limMin,
         float limMax, float antiWindup) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->antiWindup = antiWindup;
  this->tau = tau;
  this->T = T;
  this->limMin = limMin;
  this->limMax = limMax;
  this->proportional = 0.0f;
  this->integrator = 0.0f;
  this->prevError = 0.0f;
  this->differentiator = 0.0f;
  this->prevMeasurement = 0.0f;
  this->out = 0.0f;
};

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
  float error = calculateError(setpoint, measurement);

  /*
   * Proportional term
   */
  this->calculateProportionalTerm(error);

  /*
   * Integral term
   */
  this->calculateIntegralTerm(error);

  /*
   * Derivative term , derivative on measurement
   */
  this->calculateDerivativeTerm(measurement);

  /*
   * Anti-wind-up integrator
   */
  this->antiWindUp();

  /*
   * Compute output and apply limits
   */
  this->computeOutput();

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

float PID::calculateError(float setpoint, float measurement) { return setpoint - measurement; }

void PID::calculateProportionalTerm(float error) { this->proportional = this->Kp * error; }

void PID::calculateIntegralTerm(float error) {
  this->integrator = this->integrator + 0.5f * this->Ki * this->T * (error + this->prevError);
}

void PID::antiWindUp() {
  float curOutput = this->proportional + this->integrator + this->differentiator;

  if (curOutput > this->limMax || curOutput < this->limMin) {
    this->integrator = this->integrator * this->antiWindup;
  }
}

void PID::calculateDerivativeTerm(float measurement) {
  this->differentiator = (-2.0f * this->Kd * (measurement - this->prevMeasurement) +
                          (2.0f * this->tau - this->T) * this->differentiator) /
                         (2.0f * this->tau + this->T);
}

void PID::computeOutput() {
  this->out = this->proportional + this->integrator + this->differentiator;

  if (this->out > this->limMax) {
    this->out = this->limMax;
  } else if (this->out < this->limMin) {
    this->out = this->limMin;
  }
}