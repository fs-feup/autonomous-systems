#pragma once

#include <cmath>
#include <string>

#include "sensors/sensors_base.hpp"

/**
 * @brief One simulated IMU reading in the vehicle frame.
 */
struct ImuMeasurement {
  double acceleration_x = 0.0;  ///< Longitudinal acceleration (m/s^2)
  double acceleration_y = 0.0;  ///< Lateral acceleration (m/s^2)
  double yaw_rate = 0.0;        ///< Yaw rate about the vertical axis (rad/s)
};

/**
 * @brief Simulates IMU error characteristics.
 *
 * Applies a constant bias, a scale factor error proportional to the measured value, and white
 * Gaussian noise, to the accelerometer and gyroscope channels.
 */
class IMU : public Sensor {
public:
  /**
   * @brief Construct a new IMU object and load configuration.
   * @param config_path Path to the imu.yaml configuration file.
   */
  explicit IMU(const std::string& config_path);

  /**
   * @brief Corrupt a ground-truth reading with the configured error model.
   *
   * @param acceleration_x True longitudinal acceleration (m/s^2).
   * @param acceleration_y True lateral acceleration (m/s^2).
   * @param yaw_rate True yaw rate (rad/s).
   * @return The measurement an IMU with these error characteristics would report.
   */
  ImuMeasurement apply_imu_error(double acceleration_x, double acceleration_y, double yaw_rate);

private:
  // Accelerometer parameters
  double accelerometer_noise_std_dev_ = 0.0;  ///< Gaussian noise std dev (m/s^2)
  double acceleration_scale_factor_ = 0.0;    ///< Scale error coefficient (dimensionless)
  double accelerometer_bias_x_ = 0.0;         ///< Bias in X (m/s^2)
  double accelerometer_bias_y_ = 0.0;         ///< Bias in Y (m/s^2)

  // Gyroscope parameters
  double gyroscope_noise_std_dev_ = 0.0;        ///< Gaussian noise std dev (rad/s)
  double angular_velocity_scale_factor_ = 0.0;  ///< Scale error coefficient (dimensionless)
  double gyroscope_bias_ = 0.0;                 ///< Bias (rad/s)
};
