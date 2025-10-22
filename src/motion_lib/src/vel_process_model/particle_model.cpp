#include "motion_lib/vel_process_model/particle_model.hpp"

Eigen::Vector3d CAParticleModel::get_next_velocities(const Eigen::Vector3d& previous_velocities,
                                                     const Eigen::Vector3d& imu_data,
                                                     const double time_interval) {
  Eigen::Vector3d next_velocities;
  next_velocities(0) =
      previous_velocities(0) +
      (imu_data(0) + previous_velocities(1) * previous_velocities(2)) * time_interval;
  next_velocities(1) =
      previous_velocities(1) +
      (imu_data(1) - previous_velocities(0) * previous_velocities(2)) * time_interval;
  next_velocities(2) = previous_velocities(2);

  return next_velocities;
}

Eigen::Matrix3d CAParticleModel::get_jacobian_velocities(
    const Eigen::Vector3d& previous_velocities, [[maybe_unused]] const Eigen::Vector3d& imu_data,
    const double time_interval) {
  Eigen::Matrix3d jacobian_matrix = Eigen::Matrix3d::Identity();
  jacobian_matrix(0, 1) = previous_velocities(2) * time_interval;
  jacobian_matrix(0, 2) = previous_velocities(1) * time_interval;
  jacobian_matrix(1, 0) = -previous_velocities(2) * time_interval;
  jacobian_matrix(1, 2) = -previous_velocities(0) * time_interval;

  return jacobian_matrix;
}

Eigen::Matrix3d CAParticleModel::get_jacobian_sensor_data(
    [[maybe_unused]] const Eigen::Vector3d& previous_velocities,
    [[maybe_unused]] const Eigen::Vector3d& accelerations, const double time_interval) {
  Eigen::Matrix3d jacobian_matrix = Eigen::Matrix3d::Zero();
  jacobian_matrix(0, 0) = time_interval;
  jacobian_matrix(1, 1) = time_interval;
  return jacobian_matrix;
}