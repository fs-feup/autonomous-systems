#pragma once

#include <string>

#include "sensors/sensors_base.hpp"

/**
 * @brief Steering-angle sensor and motor resolver simulator
 *
 * Bias, magnitude-proportional scale error and white noise, plus encoder quantization.
 */
class Drivetrain : public Sensor {
public:
  /**
   * @brief Construct the drivetrain sensors and load their configuration
   *
   * @param config_path Path to the drivetrain.yaml configuration file
   */
  explicit Drivetrain(const std::string& config_path);

  /**
   * @brief Apply the steering sensor error model
   *
   * @param steering_angle Ground truth steering angle (rad)
   * @return double Measured steering angle (rad)
   */
  double apply_steering_error(double steering_angle);

  /**
   * @brief Apply the resolver error model
   *
   * @param motor_rpm Ground truth motor speed (rpm)
   * @return double Measured motor speed (rpm)
   */
  double apply_resolver_error(double motor_rpm);

private:
  // Steering angle sensor
  double steering_noise_std_dev_ = 0.0;  ///< Gaussian noise std dev (rad)
  double steering_scale_factor_ = 0.0;   ///< Scale error coefficient (dimensionless)
  double steering_bias_ = 0.0;           ///< Bias (rad)
  double steering_resolution_ = 0.0;     ///< Encoder resolution, 0 disables quantization (rad)

  // Motor resolver
  double resolver_noise_std_dev_ = 0.0;  ///< Gaussian noise std dev (rpm)
  double resolver_scale_factor_ = 0.0;   ///< Scale error coefficient (dimensionless)
  double resolver_bias_ = 0.0;           ///< Bias (rpm)
};
