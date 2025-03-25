#include "motion_lib/particle_model.hpp"

#include <iostream>

void CAParticleModel::update_velocities(Eigen::Vector3d& velocities, double ax, double ay,
                                        double angular_velocity, double time_interval) {
  velocities(0) += ax * time_interval;
  // velocities(1) += ay * time_interval;
  velocities(2) = angular_velocity;
}

Eigen::Matrix3d CAParticleModel::jacobian_of_velocity_update() {
  return Eigen::Matrix3d::Identity();
}
