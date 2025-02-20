#include "common_lib/structures/pose.hpp"

namespace common_lib::structures {

Pose::Pose(Position position, double orientation) : position(position), orientation(orientation) {}

Pose::Pose(double x, double y, double theta) : position(x, y), orientation(theta) {}

}  // namespace common_lib::structures