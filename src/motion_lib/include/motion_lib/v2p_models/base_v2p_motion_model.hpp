#pragma once

#include <Eigen/Dense>

class BaseV2PMotionModel {
  Eigen::Vector3f
      _base_process_noise_;  //< Equivalent to the diagonal of the process noise covariance matrix

public:
  explicit BaseV2PMotionModel();

  explicit BaseV2PMotionModel(Eigen::Vector3f base_process_noise);

  virtual ~BaseV2PMotionModel() = default;

  /**
   * @brief Predict the pose of the robot given the velocities
   *
   * @param velocities
   */
  virtual Eigen::Vector3f get_next_pose(const Eigen::Vector3f &previous_pose,
                                        const Eigen::Vector3f &velocities,
                                        const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model
   */
  virtual Eigen::Matrix3f get_jacobian(const Eigen::Vector3f &previous_pose,
                                       const Eigen::Vector3f &velocities, const double delta_t) = 0;

  /**
   * @brief Get the process noise matrix
   */
  Eigen::Matrix3f get_base_process_noise_matrix();

  /**
   * @brief Get the process noise vector
   */
  Eigen::Vector3f get_base_process_noise();
};