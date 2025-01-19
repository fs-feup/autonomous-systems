#include "slam_solver/ekf_slam/ekf_slam_solver.hpp"

#include "motion_lib/particle_model.hpp"

void EKFSLAMSolver::add_motion_prior(const common_lib::structures::Velocities& velocities) {
  if (velocities_received_ && cones_receieved_) {
    predict(this->state_, this->covariance_, process_noise_matrix_, this->last_update_, velocities);
  } else {
    velocities_received_ = true;
    this->last_update_ = std::chrono::high_resolution_clock::now();
  }
}

void EKFSLAMSolver::predict(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
                            const Eigen::MatrixXd& process_noise_matrix,
                            const std::chrono::high_resolution_clock::time_point last_update,
                            const common_lib::structures::Velocities& velocities) {
  auto current_time = std::chrono::high_resolution_clock::now();
  auto time_interval =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_update).count();

  Eigen::MatrixXd jacobian =
      motion_lib::particle_model::jacobian_of_pose_update(static_cast<int>(state.size()));

  motion_lib::particle_model::update_pose(state, velocities.velocity_x, velocities.velocity_y,
                                          velocities.rotational_velocity, time_interval);
  covariance = jacobian * covariance * jacobian.transpose() + process_noise_matrix;
}