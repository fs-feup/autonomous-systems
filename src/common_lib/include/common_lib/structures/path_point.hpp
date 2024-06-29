#pragma once

#include <functional>

#include "common_lib/structures/position.hpp"

namespace common_lib::structures {

struct PathPoint {
  Position position;
  double ideal_velocity = 1.0;
  PathPoint() = default;
  PathPoint(Position position, double ideal_velocity = 1.0);
  PathPoint(double x, double y, double ideal_velocity = 1.0);
  PathPoint(PathPoint const& path_point) = default;
  double getX() const;
  double getY() const;
  double getV() const;
  friend bool operator==(const PathPoint& lhs, const PathPoint& rhs) {
    return lhs.position == rhs.position && lhs.ideal_velocity == rhs.ideal_velocity;
  }
};

}  // namespace common_lib::structures

/**
 * @brief Hash function for path points
 *
 */
namespace std {

template <>
struct hash<common_lib::structures::PathPoint> {
  size_t operator()(const common_lib::structures::PathPoint& path_point) const noexcept {
    size_t hash_value = 0;
    // Hash the position
    hash_value ^= hash<common_lib::structures::Position>()(path_point.position);
    // Hash the ideal_velocity
    hash_value ^= hash<double>()(path_point.ideal_velocity);
    return hash_value;
  }
};

}  // namespace std