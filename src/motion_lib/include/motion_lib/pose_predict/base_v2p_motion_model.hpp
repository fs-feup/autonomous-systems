#pragma once

#include <Eigen/Dense>

#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/velocities.hpp"

class BaseV2PMotionModel {
  Eigen::Vector3f
      _process_noise_;  //< Equivalent to the diagonal of the process noise covariance matrix

public:
  explicit BaseV2PMotionModel(Eigen::Vector3f process_noise);

  virtual ~BaseV2PMotionModel() = default;

  /**
   * @brief Predict the pose of the robot given the velocities
   *
   * @param velocities
   */
  virtual common_lib::structures::Pose get_next_pose(
      const common_lib::structures::Pose previous_pose,
      common_lib::structures::Velocities &velocities, const double time_interval) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model
   */
  virtual Eigen::Matrix3f get_jacobian(const common_lib::structures::Pose &pose,
                                       const common_lib::structures::Velocities &velocities,
                                       const double time_interval) = 0;

  /**
   * @brief Get the process noise matrix
   */
  Eigen::Matrix3f get_process_noise_matrix();

  /**
   * @brief Get the process noise vector
   */
  Eigen::Vector3f get_process_noise();
};