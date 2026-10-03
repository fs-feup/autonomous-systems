#include "sensors/imu.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "config/yaml_helpers.hpp"

IMU::IMU(const std::string& config_path) {
  using invictasim::config::load_optional;

  const YAML::Node imu = YAML::LoadFile(config_path)["imu_sensor"];

  const YAML::Node accelerometer = imu["accelerometer"];
  load_optional(accelerometer, "noise_std_dev", accelerometer_noise_std_dev_);
  load_optional(accelerometer, "acceleration_scale_factor", acceleration_scale_factor_);
  load_optional(accelerometer, "bias_x", accelerometer_bias_x_);
  load_optional(accelerometer, "bias_y", accelerometer_bias_y_);

  const YAML::Node gyroscope = imu["gyroscope"];
  load_optional(gyroscope, "noise_std_dev", gyroscope_noise_std_dev_);
  load_optional(gyroscope, "angular_velocity_scale_factor", angular_velocity_scale_factor_);
  load_optional(gyroscope, "bias", gyroscope_bias_);
}

ImuMeasurement IMU::apply_imu_error(double acceleration_x, double acceleration_y, double yaw_rate) {
  ImuMeasurement measurement;

  // Scale factor error is proportional to the signed measurement, so it grows with magnitude and
  // keeps the sign of what it is distorting.
  measurement.acceleration_x = acceleration_x * (1.0 + acceleration_scale_factor_) +
                               accelerometer_bias_x_ + gaussian_noise(accelerometer_noise_std_dev_);
  measurement.acceleration_y = acceleration_y * (1.0 + acceleration_scale_factor_) +
                               accelerometer_bias_y_ + gaussian_noise(accelerometer_noise_std_dev_);
  measurement.yaw_rate = yaw_rate * (1.0 + angular_velocity_scale_factor_) + gyroscope_bias_ +
                         gaussian_noise(gyroscope_noise_std_dev_);

  return measurement;
}
