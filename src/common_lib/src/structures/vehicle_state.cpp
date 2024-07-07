#include "common_lib/structures/vehicle_state.hpp"

namespace common_lib::structures {

VehicleState::VehicleState(Pose pose, double linear_velocity,
                           double rotational_velocity)
    : pose(std::move(pose)),
      linear_velocity(linear_velocity),
      rotational_velocity(rotational_velocity) {}

VehicleState::VehicleState(double x, double y, double theta, double linear_velocity,
                           double rotational_velocity)
    : pose(Pose(x, y, theta)),
      linear_velocity(linear_velocity),
      rotational_velocity(rotational_velocity) {}

}  // namespace common_lib::structures