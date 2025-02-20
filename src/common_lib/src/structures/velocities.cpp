#include "common_lib/structures/velocities.hpp"

namespace common_lib::structures {

Velocities::Velocities(double velocity_x, double velocity_y, double rotational_velocity)
    : velocity_x(velocity_x), velocity_y(velocity_y), rotational_velocity(rotational_velocity) {}

}  // namespace common_lib::structures