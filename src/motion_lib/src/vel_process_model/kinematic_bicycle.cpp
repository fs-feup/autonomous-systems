#include "motion_lib/vel_process_model/kinematic_bicycle.hpp"

Eigen::Vector3d KinematicBicycleProcess::get_next_velocities(
    const Eigen::Vector3d& previous_velocities, const Eigen::Vector3d& measurements,
    const double time_interval) {
  Eigen::Vector3d next_velocities;
  next_velocities(0) =
      previous_velocities(0) +
      (measurements(0) + previous_velocities(1) * previous_velocities(2)) * time_interval;
  next_velocities(1) =
      previous_velocities(1) +
      (measurements(1) - previous_velocities(0) * previous_velocities(2)) * time_interval;
  next_velocities(2) =
      previous_velocities(0) * std::tan(measurements(2)) / this->_params_->wheelbase;

  return next_velocities;
}

Eigen::Matrix3d KinematicBicycleProcess::get_jacobian_velocities(
    const Eigen::Vector3d& previous_velocities,
    [[maybe_unused]] const Eigen::Vector3d& measurements, const double time_interval) {
  Eigen::Matrix3d jacobian_matrix = Eigen::Matrix3d::Identity();
  jacobian_matrix(0, 1) = previous_velocities(2) * time_interval;
  jacobian_matrix(0, 2) = previous_velocities(1) * time_interval;
  jacobian_matrix(1, 0) = -previous_velocities(2) * time_interval;
  jacobian_matrix(1, 2) = -previous_velocities(0) * time_interval;
  jacobian_matrix(2, 0) = std::tan(measurements(2)) / this->_params_->wheelbase;
  jacobian_matrix(2, 2) = 0;

  return jacobian_matrix;
}