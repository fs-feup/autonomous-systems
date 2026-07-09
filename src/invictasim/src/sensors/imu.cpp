#include "sensors/imu.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>

IMU::IMU(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node imu = config["imu_sensor"];

  // Load accelerometer parameters
  YAML::Node accelerometer = imu["accelerometer"];
  accelerometer_noise_std_dev_ = accelerometer["noise_std_dev"].as<double>();
  acceleration_scale_factor_ = accelerometer["acceleration_scale_factor"].as<double>();
  accelerometer_bias_x_ = accelerometer["bias_x"].as<double>();
  accelerometer_bias_y_ = accelerometer["bias_y"].as<double>();

  // Load gyroscope parameters
  YAML::Node gyroscope = imu["gyroscope"];
  gyroscope_noise_std_dev_ = gyroscope["noise_std_dev"].as<double>();
  angular_velocity_scale_factor_ = gyroscope["angular_velocity_scale_factor"].as<double>();
  gyroscope_bias_ = gyroscope["bias"].as<double>();
}

IMU::IMUMeasurement IMU::apply_imu_error(double acceleration_x, double acceleration_y, double yaw_rate) 
{
  IMUMeasurement measurement;
  // Apply accelerometer error modeling
  // Add bias
  double acc_x_with_bias = Sensor::apply_bias(acceleration_x, accelerometer_bias_x_);
  double acc_y_with_bias = Sensor::apply_bias(acceleration_y, accelerometer_bias_y_);

  // Apply scale factor error that depends on acceleration magnitude
  // error = scale_factor * acceleration_magnitude
  double scale_error_x = acceleration_scale_factor_ * acceleration_x;
  double scale_error_y = acceleration_scale_factor_ * acceleration_y;

  acc_x_with_bias += scale_error_x;
  acc_y_with_bias += scale_error_y;

  // Apply white Gaussian noise
  measurement.acceleration_x = acc_x_with_bias + gaussian_noise(accelerometer_noise_std_dev_);
  measurement.acceleration_y = acc_y_with_bias + gaussian_noise(accelerometer_noise_std_dev_);

  // Apply gyroscope error modeling
  // Add bias
  double yaw_rate_with_bias = Sensor::apply_bias(yaw_rate, gyroscope_bias_);

  // Apply scale factor error that depends on angular velocity magnitude
  double scale_error_angular = angular_velocity_scale_factor_ * yaw_rate * std::abs(yaw_rate);

  yaw_rate_with_bias += scale_error_angular;

  // Apply white Gaussian noise
  measurement.angular_velocity_z = yaw_rate_with_bias + gaussian_noise(gyroscope_noise_std_dev_);

  return measurement;
}