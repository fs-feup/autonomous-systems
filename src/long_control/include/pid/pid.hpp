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
    public:
  float Kp; /**< Proporcional gain */
  float Ki; /**< Integral gain */
  float Kd; /**< Derivative gain */

  float tau; /**< Derivative low pass filter time constant */

  float T; /**< Sampling time */

  float limMin; /**< Minimum output value */
  float limMax; /**< Maximum output value */

  float integrator;      /**< Integrator value */
  float prevError;       /**< Previous error value, required for integrator */
  float differentiator;  /**< Differentiator value */
  float prevMeasurement; /**< Previous measurement value, required for defferentiator */

  float out; /**< Output value */

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
     */
    PID(float Kp, float Ki, float Kd, float tau, float T, float limMin, float limMax);

    /**
     * @brief Calculate the error signal
     *
     * @param setpoint
     * @param measurement
     * @return error (float)
     */
    float calculateError(float setpoint, float measurement);

    /**
     * @brief Calculate the proportional term
     *
     * @param Kp
     * @param error
     * @return float
     */
    float calculateProportionalTerm(float Kp, float error);


    /**
     * @brief Calculate the integral term
     *
     * @param pid
     * @param error
     */
    void calculateIntegralTerm(PID* pid, float error);

    /**
     * @brief Calculate the derivative term
     *
     * @param pid
     * @param measurement
     */
    float antiWindUp(PID *pid,float proportional);
    
    /**
     * @brief Calculate the derivative term
     *
     * @param pid
     * @param measurement
     */
    void calculateIntegralTerm(PID* pid, float error);

    /**
     * @brief Calculate the derivative term (derivative on measurement)
     *
     * @param pid
     * @param measurement
     */
    void calculateDerivativeTerm(PID *pid, float measurement);

    /**
     * @brief Compute the output value and apply limits
     *
     * @param pid
     * @param proportional
     */
    void computeOutput(PID *pid, float proportional);

};
#endif  // PID_CONTROLLER_HPP_