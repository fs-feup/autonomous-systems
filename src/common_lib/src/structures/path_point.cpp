#include "common_lib/structures/path_point.hpp"

namespace common_lib::structures {

PathPoint::PathPoint(Position position, double orientation, double ideal_velocity)
    : position(position), orientation(orientation), ideal_velocity(ideal_velocity) {}

PathPoint::PathPoint(double x, double y, double orientation, double ideal_velocity)
    : position(x, y), orientation(orientation), ideal_velocity(ideal_velocity) {}

}  // namespace common_lib::structures
