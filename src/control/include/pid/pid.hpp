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

  double proportional{0.0f};   /**< Integrator value */
  double integrator{0.0f};     /**< Integrator value */
  double differentiator{0.0f}; /**< Differentiator value */

  double prevError{0.0f};       /**< Previous error value, required for integrator */
  double prevMeasurement{0.0f}; /**< Previous measurement value, required for defferentiator */

  double out{0.0f}; /**< Output value */

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
  double calculate_error(double setpoint, double measurement) const;

  /**
   * @brief Calculate the proportional term
   *
   * @param error
   */
  void calculate_proportional_term(double error);

  /**
   * @brief Calculate the integral term
   *
   * @param error
   */
  void calculate_integral_term(double error);

  /**
   * @brief Anti-wind-up via dynamic integrator clamping
   */
  void anti_wind_up();

  /**
   * @brief Calculate the derivative term (derivative on measurement)
   *
   * @param measurement
   */
  void calculate_derivative_term(double measurement);

  /**
   * @brief Compute the output value and apply limits
   */
  void compute_output();
};