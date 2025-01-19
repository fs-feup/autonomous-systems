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

  /**
   * @brief Executed to deal with new velocity data
   *
   * @param velocities
   */
  void add_motion_prior(const common_lib::structures::Velocities& velocities) override;

  /**
   * @brief executed when velocity data is received. Prediction step of the EKF
   * which is meant to capture changes in the state of the system
   *
   * @param state vector with position and orientation, followed by the landmark positions {car_x,
   * car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param covariance
   * @param process_noise_matrix estimated process noise
   * @param last_update last time velocity data was received
   * @param velocities new velocity data
   */
  void predict(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
               const Eigen::MatrixXd& process_noise_matrix,
               const std::chrono::high_resolution_clock::time_point last_update,
               const common_lib::structures::Velocities& velocities);
};