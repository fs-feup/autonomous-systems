#pragma once

#include <rclcpp/rclcpp.hpp>

namespace common_lib::sensor_data {

/**
 * @brief Struct for wheel encoder data
 *
 * @param left_ticks Number of ticks on the left wheel
 * @param right_ticks Number of ticks on the right wheel
 */
struct WheelEncoderData {
  double rl_rpm = 0.0;
  double rr_rpm = 0.0;
  double fl_rpm = 0.0;
  double fr_rpm = 0.0;
  double steering_angle = 0.0;

  rclcpp::Time timestamp_ = rclcpp::Time(0);  //< Time of sensor data collection

  WheelEncoderData() = default;
  WheelEncoderData(double rl_rpm, double rr_rpm, double steering_angle,
                   rclcpp::Time timestamp = rclcpp::Time(0));
  WheelEncoderData(double rl_rpm, double rr_rpm, double fl_rpm, double fr_rpm,
                   double steering_angle, rclcpp::Time timestamp = rclcpp::Time(0));
};

}  // namespace common_lib::sensor_data