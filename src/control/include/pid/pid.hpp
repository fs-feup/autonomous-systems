#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

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
  float Kp;         /**< Proporcional gain */
  float Ki;         /**< Integral gain */
  float Kd;         /**< Derivative gain */
  float antiWindup; /**< Gain of integrator impact when saturated */

  float tau; /**< Derivative low pass filter time constant */

  float T; /**< Sampling time */

  float limMin; /**< Minimum output value */
  float limMax; /**< Maximum output value */

  float proportional{0.0f};   /**< Integrator value */
  float integrator{0.0f};     /**< Integrator value */
  float differentiator{0.0f}; /**< Differentiator value */

  float prevError{0.0f};       /**< Previous error value, required for integrator */
  float prevMeasurement{0.0f}; /**< Previous measurement value, required for defferentiator */

  float out{0.0f}; /**< Output value */

  /**
   * @brief Calculate the output value
   *
   * @param setpoint
   * @param measurement
   * @return float
   */
  float update(float setpoint, float measurement);

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
  PID(float Kp, float Ki, float Kd, float tau, float T, float limMin, float limMax,
      float antiWindup);

  /**
   * @brief Calculate the error signal
   *
   * @param setpoint
   * @param measurement
   * @return error (float)
   */
  float calculate_error(float setpoint, float measurement) const;

  /**
   * @brief Calculate the proportional term
   *
   * @param error
   */
  void calculate_proportional_term(float error);

  /**
   * @brief Calculate the integral term
   *
   * @param error
   */
  void calculate_integral_term(float error);

  /**
   * @brief Anti-wind-up via dynamic integrator clamping
   */
  void anti_wind_up();

  /**
   * @brief Calculate the derivative term (derivative on measurement)
   *
   * @param measurement
   */
  void calculate_derivative_term(float measurement);

  /**
   * @brief Compute the output value and apply limits
   */
  void compute_output();
};
#endif  // PID_CONTROLLER_HPP_