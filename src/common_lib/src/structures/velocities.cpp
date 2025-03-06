#include "common_lib/structures/velocities.hpp"

namespace common_lib::structures {

Velocities::Velocities(double velocity_x, double velocity_y, double rotational_velocity,
                       double velocity_x_noise, double velocity_y_noise,
                       double rotational_velocity_noise, rclcpp::Time timestamp)
    : velocity_x(velocity_x),
      velocity_y(velocity_y),
      rotational_velocity(rotational_velocity),
      velocity_x_noise_(velocity_x_noise),
      velocity_y_noise_(velocity_y_noise),
      rotational_velocity_noise_(rotational_velocity_noise),
      timestamp(timestamp) {}

}  // namespace common_lib::structures