#include "common_lib/structures/path_point.hpp"

namespace common_lib::structures {  

PathPoint::PathPoint(Position position, double ideal_velocity) : position(position), ideal_velocity(ideal_velocity) {}

PathPoint::PathPoint(double x, double y, double ideal_velocity) : position(x, y), ideal_velocity(ideal_velocity) {}

PathPoint::PathPoint(PathPoint const &path_point) : position(path_point.position), ideal_velocity(path_point.ideal_velocity) {}

} // namespace common_lib::structures   