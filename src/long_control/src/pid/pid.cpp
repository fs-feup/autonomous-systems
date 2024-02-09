#include "pid/pid.hpp"

/**
 * @brief Construct a new PID object
 *
 * @param Kp Proporcional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param tau Derivative low pass filter time constant
 * @param T Sampling time
 * @param limMin Minimum output value
 * @param limMax Maximum output value
 */

PID::PID(float Kp, float Ki, float Kd, float tau, float T, float limMin, float limMax) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->tau = tau;
  this->T = T;
  this->limMin = limMin;
  this->limMax = limMax;
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
  float proportional = calculateProportionalTerm(this->Kp, error);

  /*
   * Integral term
   */
  calculateIntegralTerm(this,error);

  /*
   * Anti-wind-up via dynamic integrator clamping
   */
  antiWindUp(this, proportional);

  /*
   * Derivative term , derivative on measurement
   */
  calculateDerivativeTerm(this,measurement);

  /*
   * Compute output and apply limits
   */
  computeOutput(this,proportional);  

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

float calculateError(float setpoint, float measurement) { return setpoint - measurement; }

float calculateProportionalTerm(float Kp, float error) { return Kp * error; }

void calculateIntegralTerm(PID* pid, float error) {
  pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
}

float antiWindUp(PID *pid,float proportional) {
  float limMinInt; /* lim min integrator term */
  float limMaxInt; /* lim max integrator term */


  /* Compute integrator limits */
  if (pid->limMax > proportional) {
    limMaxInt = pid->limMax - proportional;
  } else {
    limMaxInt = 0.0f;  // anti-wind-up is active
  }

  if (pid->limMin < proportional) {
    limMinInt = pid->limMin - proportional;
  } else {
    limMinInt = 0.0f;  // anti-wind-up is active
  }

  /*
   * Clamp the integrator
   */
  if (pid->integrator > limMaxInt) {
    pid->integrator = limMaxInt;
  } else if (pid->integrator < limMinInt) {
    pid->integrator = limMinInt;
  }
}

void calculateDerivativeTerm(PID *pid, float measurement) {
  pid->differentiator = (-2.0f * pid->Kd * (measurement - pid->prevMeasurement) +
                          (2.0f * pid->tau - pid->T) * pid->differentiator) /
                         (2.0f * pid->tau + pid->T);
}

void computeOutput(PID *pid, float proportional) {
  pid->out = proportional + pid->integrator + pid->differentiator;

  if (pid->out > pid->limMax) {
    pid->out = pid->limMax;
  } else if (pid->out < pid->limMin) {
    pid->out = pid->limMin;
  }
}