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
};

bool operator==(const PathPoint& lhs, const PathPoint& rhs);
}  // namespace common_lib::structures

/**
 * @brief Hash function for path points
 *
 */
namespace std {

template <>
struct hash<common_lib::structures::PathPoint> {
  size_t operator()(const common_lib::structures::PathPoint& pathPoint) const noexcept {
    size_t hashValue = 0;
    // Hash the position
    hashValue ^= hash<common_lib::structures::Position>()(pathPoint.position);
    // Hash the ideal_velocity
    hashValue ^= hash<double>()(pathPoint.ideal_velocity);
    return hashValue;
  }
};

}  // namespace std