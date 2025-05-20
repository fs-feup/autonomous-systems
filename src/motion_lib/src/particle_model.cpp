#include "motion_lib/particle_model.hpp"

#include <iostream>

void CAParticleModel::update_velocities(Eigen::Vector3d& velocities, double ax, double ay,
                                        double angular_velocity, double time_interval) {
  double original_velocity_0 = velocities(0);
  velocities(0) += (ax + velocities(1) * angular_velocity) * time_interval;
  velocities(1) += (ay - original_velocity_0 * angular_velocity) * time_interval;
  velocities(2) = angular_velocity;
}

Eigen::Matrix3d CAParticleModel::jacobian_of_velocity_update() {
  return Eigen::Matrix3d::Identity();
}
