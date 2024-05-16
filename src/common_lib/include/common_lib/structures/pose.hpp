#pragma once

#include "common_lib/structures/position.hpp"

namespace common_lib::structures {
/**
 * @brief Struct for pose representation
 *
 * @param position Vehicle coordinates, x and y
 * @param orientation Orientation of the vehicle in radians
 * 0 radians is pointing in the positive x direction
 */
struct Pose {
  Position position;
  double orientation = 0.0;  /// theta in radians
  Pose() = default;
  Pose(Position position, double orientation);
  Pose(double x, double y, double theta);
};
}  // namespace common_lib::structures