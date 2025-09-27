#pragma once
#include <memory>

#include "config/parameters.hpp"
#include "gtest/gtest.h"

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
private:
  std::shared_ptr<ControlParameters> params_;
  double kp_;          /**< Proporcional gain */
  double ki_;          /**< Integral gain */
  double kd_;          /**< Derivative gain */
  double anti_windup_; /**< Gain of integrator impact when saturated */

  double tau_; /**< Derivative low pass filter time constant */

  double t_; /**< Sampling time */

  double lim_min_; /**< Minimum output value */
  double lim_max_; /**< Maximum output value */

  double proportional_{0.0f};   /**< Integrator value */
  double integrator_{0.0f};     /**< Integrator value */
  double differentiator_{0.0f}; /**< Differentiator value */

  double prev_error_{0.0f};       /**< Previous error value, required for integrator */
  double prev_measurement_{0.0f}; /**< Previous measurement value, required for defferentiator */

  double out_{0.0f}; /**< Output value */

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

public:
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
   * @param antiWindup Anti-windup constant
   */
  PID(const ControlParameters &params);

  /**
   * @brief PID default constructor
   */
  PID();

  /**
   * @brief Calculate the output value
   *
   * @param setpoint
   * @param measurement
   * @return double
   */
  double update(double setpoint, double measurement);

  FRIEND_TEST(PidTests, TestAntiWindUp1);
  FRIEND_TEST(PidTests, TestAntiWindUp2);
  FRIEND_TEST(PidTests, TestAntiWindUp3);
  FRIEND_TEST(PidTests, ProportionalTerm);
  FRIEND_TEST(PidTests, IntegralTerm1);
  FRIEND_TEST(PidTests, IntegralTerm2);
  FRIEND_TEST(PidTests, DerivativeTerm1);
  FRIEND_TEST(PidTests, DerivativeTerm2);
  FRIEND_TEST(PidTests, Output1);
  FRIEND_TEST(PidTests, Output2);
  FRIEND_TEST(PidTests, Output3);
  FRIEND_TEST(PidTests, Update1);
};