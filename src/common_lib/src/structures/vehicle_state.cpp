#include "common_lib/structures/vehicle_state.hpp"

namespace common_lib::structures {

VehicleState::VehicleState(Pose pose, double velocity_x, double velocity_y,
                           double rotational_velocity)
    : pose(std::move(pose)),
      velocity_x(velocity_x),
      velocity_y(velocity_y),
      rotational_velocity(rotational_velocity) {}

VehicleState::VehicleState(double x, double y, double theta, double velocity_x, double velocity_y,
                           double rotational_velocity)
    : pose(Pose(x, y, theta)),
      velocity_x(velocity_x),
      velocity_y(velocity_y),
      rotational_velocity(rotational_velocity) {}

}  // namespace common_lib::structures