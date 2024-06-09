#include "common_lib/structures/path_point.hpp"

namespace common_lib::structures {

PathPoint::PathPoint(Position position, double ideal_velocity)
    : position(position), ideal_velocity(ideal_velocity) {}

PathPoint::PathPoint(double x, double y, double ideal_velocity)
    : position(x, y), ideal_velocity(ideal_velocity) {}

bool operator==(const PathPoint& lhs, const PathPoint& rhs) {
  return lhs.position == rhs.position && lhs.ideal_velocity == rhs.ideal_velocity;
}
}  // namespace common_lib::structures
