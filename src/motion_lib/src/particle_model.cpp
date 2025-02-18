#include "motion_lib/particle_model.hpp"

namespace motion_lib::particle_model {

void update_velocities(Eigen::Vector3f& velocities, double ax, double ay, double angular_velocity,
                       double time_interval) {
  velocities(0) += ax * time_interval;
  velocities(1) += ay * time_interval;
  velocities(2) += angular_velocity;
}

Eigen::Matrix3f jacobian_of_velocity_update() { return Eigen::Matrix3f::Identity(); }
}  // namespace motion_lib::particle_model