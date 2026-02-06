#pragma once

#include <rclcpp/rclcpp.hpp>

namespace common_lib::structures {

struct Wheels {
  double front_left = 0.0;
  double front_right = 0.0;
  double rear_left = 0.0;
  double rear_right = 0.0;

  /**
   * @brief Construct a new Wheels object with default values
   */
  Wheels() = default;

  /**
   * @brief Construct a new Wheels object with the given velocities and noise
   *
   * @param front_left front left wheel
   * @param front_right front right wheel
   * @param rear_left rear left wheel
   * @param rear_right rear right wheel
   */
  Wheels(double front_left, double front_right, double rear_left, double rear_right);
};

}  // namespace common_lib::structures