#pragma once

#include <rclcpp/rclcpp.hpp>

namespace common_lib::structures {

struct Velocities {
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double rotational_velocity = 0.0;

  double velocity_x_noise_ = 0.0;
  double velocity_y_noise_ = 0.0;
  double rotational_velocity_noise_ = 0.0;

  rclcpp::Time timestamp_ = rclcpp::Time(0);  //< Last time the velocities were updated

  /**
   * @brief Construct a new Velocities object with default values
   */
  Velocities() = default;

  /**
   * @brief Construct a new Velocities object with the given velocities and noise
   *
   * @param velocity_x x velocity
   * @param velocity_y y velocity
   * @param rotational_velocity rotational velocity
   * @param velocity_x_noise x velocity noise
   * @param velocity_y_noise y velocity noise
   * @param rotational_velocity_noise rotational velocity noise (z axis)
   * @param timestamp timestamp of the velocities
   */
  Velocities(double velocity_x, double velocity_y, double rotational_velocity,
             double velocity_x_noise = 0.0, double velocity_y_noise = 0.0,
             double rotational_velocity_noise = 0.0, rclcpp::Time timestamp = rclcpp::Time(0));
};

}  // namespace common_lib::structures