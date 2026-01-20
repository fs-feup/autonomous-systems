#pragma once

#include <rclcpp/rclcpp.hpp>

namespace common_lib::sensor_data {

struct ImuData {
  double rotational_velocity;
  double acceleration_x;
  double acceleration_y;
  double rotational_velocity_noise;
  double acceleration_x_noise;
  double acceleration_y_noise;

  rclcpp::Time timestamp_ = rclcpp::Time(0);  //< Time of sensor data collection

  ImuData() = default;
  ImuData(double rotational_velocity, double acceleration_x, double acceleration_y,
          rclcpp::Time timestamp = rclcpp::Time(0), double rotational_velocity_noise = 0.0,
          double acceleration_x_noise = 0.0, double acceleration_y_noise = 0.0);
};
}  // namespace common_lib::sensor_data