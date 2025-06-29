#pragma once

#include <rclcpp/rclcpp.hpp>

namespace common_lib::sensor_data {

struct ImuData {
  float rotational_velocity;
  float acceleration_x;
  float acceleration_y;

  rclcpp::Time timestamp_ = rclcpp::Time(0);  //< Time of sensor data collection

  ImuData() = default;
  ImuData(float rotational_velocity, float acceleration_x, float acceleration_y,
          rclcpp::Time timestamp = rclcpp::Time(0));
};
}  // namespace common_lib::sensor_data