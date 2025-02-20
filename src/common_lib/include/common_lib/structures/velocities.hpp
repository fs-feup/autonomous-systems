#pragma once

namespace common_lib::structures {

struct Velocities {
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double rotational_velocity = 0.0;

  Velocities() = default;
  Velocities(double velocity_x, double velocity_y, double rotational_velocity);
};

}  // namespace common_lib::structures