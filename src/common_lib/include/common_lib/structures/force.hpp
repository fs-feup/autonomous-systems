#pragma once

#include <rclcpp/rclcpp.hpp>

namespace common_lib::structures {

struct Force {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  /**
   * @brief Construct a new Force object with default values
   */
  Force() = default;

  /**
   * @brief Construct a new Force object with the given forces
   *
   * @param Force x force
   * @param Force y force
   * @param Force z force
   */
  Force(double force_x, double force_y, double force_z);
};
}  // namespace common_lib::structures