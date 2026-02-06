#pragma once

#include <rclcpp/rclcpp.hpp>

namespace common_lib::structures {

struct Forces {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  /**
   * @brief Construct a new Forces object with default values
   */
  Forces() = default;

  /**
   * @brief Construct a new Forces object with the given forces
   *
   * @param Force x force
   * @param Force y force
   * @param Force z force
   */
  Forces(double force_x, double force_y, double force_z);
};
}  // namespace common_lib::structures