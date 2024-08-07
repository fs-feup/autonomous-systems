#pragma once

#include <utility>

#include "common_lib/structures/pose.hpp"

namespace common_lib::structures {

struct VehicleState {
  Pose pose;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double rotational_velocity = 0.0;

  VehicleState() = default;
  VehicleState(Pose pose, double velocity_x, double velocity_y, double rotational_velocity);
  VehicleState(double x, double y, double theta, double velocity_x, double velocity_y,
               double rotational_velocity);
};

}  // namespace common_lib::structures