#pragma once

/**
 * @brief PI-D Controller class
 *
 * @details
 * This class implements a PI-D controller.
 * Its a PI-D because the P and I terms operates on the Error signal and the D works with the
 * feedback signal (measurement).
 *
 */

class PID {
 public:            // pirvate vs public
  double Kp;         /**< Proporcional gain */
  double Ki;         /**< Integral gain */
  double Kd;         /**< Derivative gain */
  double antiWindup; /**< Gain of integrator impact when saturated */

  double tau; /**< Derivative low pass filter time constant */

  double T; /**< Sampling time */

  double limMin; /**< Minimum output value */
  double limMax; /**< Maximum output value */

  double proportional;   /**< Integrator value */
  double integrator;     /**< Integrator value */
  double differentiator; /**< Differentiator value */

  double prevError;       /**< Previous error value, required for integrator */
  double prevMeasurement; /**< Previous measurement value, required for defferentiator */

  double out; /**< Output value */

  /**
   * @brief Calculate the output value
   *
   * @param setpoint
   * @param measurement
   * @return double
   */
  double update(double setpoint, double measurement);

 public:
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
   * @param antiWindup Gain of integrator impact when saturated
   */
  PID(double Kp, double Ki, double Kd, double tau, double T, double limMin, double limMax,
      double antiWindup);

  /**
   * @brief PID default constructor
   */
  PID();

  /**
   * @brief Calculate the error signal
   *
   * @param setpoint
   * @param measurement
   * @return error (double)
   */
  double calculateError(double setpoint, double measurement);

  /**
   * @brief Calculate the proportional term
   *
   * @param error
   */
  void calculateProportionalTerm(double error);

  /**
   * @brief Calculate the integral term
   *
   * @param error
   */
  void calculateIntegralTerm(double error);

  /**
   * @brief Anti-wind-up via dynamic integrator clamping
   */
  void antiWindUp();

  /**
   * @brief Calculate the derivative term (derivative on measurement)
   *
   * @param measurement
   */
  void calculateDerivativeTerm(double measurement);

  /**
   * @brief Compute the output value and apply limits
   */
  void computeOutput();
};