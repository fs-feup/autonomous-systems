#pragma once

#include <Eigen/Dense>

class V2PMotionModel {
  Eigen::Vector3d
      _base_process_noise_;  //< Equivalent to the diagonal of the process noise covariance matrix

public:
  explicit V2PMotionModel();

  explicit V2PMotionModel(Eigen::Vector3d base_process_noise);

  virtual ~V2PMotionModel() = default;

  /**
   * @brief Predict the pose of the robot given the velocities
   *
   * @param velocities
   */
  virtual Eigen::Vector3d get_next_pose(const Eigen::Vector3d &previous_pose,
                                        const Eigen::Vector3d &velocities,
                                        const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model
   */
  virtual Eigen::Matrix3d get_jacobian(const Eigen::Vector3d &previous_pose,
                                       const Eigen::Vector3d &velocities, const double delta_t) = 0;

  /**
   * @brief Get the process noise matrix
   */
  Eigen::Matrix3d get_base_process_noise_matrix();

  /**
   * @brief Get the process noise vector
   */
  Eigen::Vector3d get_base_process_noise();
};