#include "common_lib/structures/path_point.hpp"

namespace common_lib::structures {

PathPoint::PathPoint(Position position, double ideal_velocity)
    : position(position), ideal_velocity(ideal_velocity) {}

PathPoint::PathPoint(double x, double y, double ideal_velocity)
    : position(x, y), ideal_velocity(ideal_velocity) {}

PathPoint::PathPoint(double x, double y, Cone* cone1, Cone* cone2)
    : position(x, y), cone1(cone1), cone2(cone2) {}

}  // namespace common_lib::structures
