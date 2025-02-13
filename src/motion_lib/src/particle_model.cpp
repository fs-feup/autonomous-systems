#include "motion_lib/particle_model.hpp"

#include <iostream>

void CVParticleModel::update_velocities(Eigen::Vector3d& velocities, double ax, double ay,
                                        double angular_velocity, double time_interval) {
  velocities(0) += ax * time_interval;
  velocities(1) += ay * time_interval;
  velocities(2) = angular_velocity;
}

Eigen::Matrix3d CVParticleModel::jacobian_of_velocity_update() {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  jacobian(0, 0) = 1;
  jacobian(1, 1) = 1;
  jacobian(2, 2) = 1;
  return jacobian;
}
