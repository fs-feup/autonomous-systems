#pragma once

#include <Eigen/Dense>
#include <chrono>

#include "slam_solver/slam_solver.hpp"

class EKFSLAMSolver : public SLAMSolver {
  Eigen::VectorXd state_ = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd covariance_ = Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd process_noise_matrix_ =
      Eigen::MatrixXd::Identity(3, 3) * 0.1;  // TODO: Pass value from launch file

  std::chrono::high_resolution_clock::time_point last_update_;

  bool velocities_received_ = false;
  bool cones_receieved_ = false;

  void add_motion_prior(const common_lib::structures::Velocities& velocities) override;
  void predict(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
               const Eigen::MatrixXd& process_noise_matrix,
               const std::chrono::high_resolution_clock::time_point last_update,
               const common_lib::structures::Velocities& velocities);
};