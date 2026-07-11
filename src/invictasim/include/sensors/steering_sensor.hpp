#pragma once

#include <string>

#include "sensors/sensors_base.hpp"

/**
 * @brief Simulates a steering angle sensor with bias, scale factor, and Gaussian noise.
 */
class SteeringSensor : public Sensor {
public:
  /**
   * @brief Construct a new steering angle sensor and load configuration.
   *
   * @param config_path Path to the steering angle YAML configuration file.
   */
  explicit SteeringSensor(const std::string& config_path);

  /**
   * @brief Apply steering angle sensor error modeling to a raw steering angle.
   *
   * @param steering_angle_raw Raw steering angle in radians.
   * @return double Measured steering angle after applying bias, scale, noise, and quantization.
   */
  double apply_sas_error(double steering_angle_raw);

private:
  double noise_std_dev_;        ///< Gaussian noise std dev (rad)
  double bias_;                 ///< Constant bias offset (rad)
  double quantization_step_;    ///< Quantization step size (rad)
};
