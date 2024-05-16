#ifndef SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSE_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSE_HPP_

#include <cmath>

struct Position {
  double x = 0;
  double y = 0;
  Position() = default;
  Position(double x, double y) : x(x), y(y) {}
  double distance_to(Position p) const { return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2)); }
  double distance_squared_to(Position p) const { return pow(x - p.x, 2) + pow(y - p.y, 2); }
};

struct Pose {
  Position position;
  double orientation = 0.0;
  Pose() = default;
  Pose(Position position, double orientation) : position(position), orientation(orientation) {}
  Pose(double x, double y, double theta) : position(x, y), orientation(theta) {}
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSE_HPP_