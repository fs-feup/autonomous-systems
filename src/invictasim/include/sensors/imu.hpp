#pragma once

#include <cmath>
#include <string>

#include "sensors/sensors_base.hpp"

/**
 * @brief 
 * Simulates IMU sensor characteristics including:
 * - White Gaussian noise on accelerometer and gyroscope measurements
 * - Scale factor errors that scale with acceleration magnitude
 * - Constant bias offsets
 */
class IMU : public Sensor {
public:
  /**
   * @brief IMU measurement output structure
   */
  struct IMUMeasurement {
    double acceleration_x = 0.0;      ///< Measured acceleration in X direction (m/s²)
    double acceleration_y = 0.0;      ///< Measured acceleration in Y direction (m/s²)
    double angular_velocity_z = 0.0;  ///< Measured yaw rate (rad/s)
  };

  /**
   * @brief Construct a new IMU object and load configuration
   *
   * @param config_path Path to the imu.yaml configuration file
   */
  explicit IMU(const std::string& config_path);
  /**
   * @param acceleration_x Raw acceleration in X direction (m/s²)
   * @param acceleration_y Raw acceleration in Y direction (m/s²)
   * @param yaw_rate Raw yaw rate (rad/s)
   * @return IMUMeasurement struct containing error-corrupted measurements
   */
  IMUMeasurement apply_imu_error(double acceleration_x, double acceleration_y, double yaw_rate);

  /**
   * @brief Simulate acceleration-dependent scale factor error
   *
   * The scale factor error increases with acceleration magnitude.
   * This is modeled as: error = acceleration_scale_factor * ||acceleration||
   * @param acceleration_x Acceleration in X direction (m/s²)
   * @param acceleration_y Acceleration in Y direction (m/s²)
   * @return double The scale factor error magnitude
   */
  double calculate_acceleration_scale_error(double acceleration_x, double acceleration_y);

private:
  // Accelerometer parameters
  double accelerometer_noise_std_dev_;  ///< Gaussian noise std dev (m/s²)
  double acceleration_scale_factor_;    ///< Scale error coefficient
  double accelerometer_bias_x_;         ///< Bias in X (m/s²)
  double accelerometer_bias_y_;         ///< Bias in Y (m/s²)
  double accelerometer_bias_z_;         ///< Bias in Z (m/s²)

  // Gyroscope parameters
  double gyroscope_noise_std_dev_;        ///< Gaussian noise std dev (rad/s)
  double angular_velocity_scale_factor_;  ///< Scale error coefficient
  double gyroscope_bias_;                 ///< Bias (rad/s)

  // General parameters
  double update_rate_hz_;         ///< IMU update rate
  bool temperature_sensitivity_;  ///< Flag for temperature modeling
};
