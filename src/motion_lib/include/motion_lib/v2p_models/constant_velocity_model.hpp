#pragma once

#include <Eigen/Sparse>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

/**
 * @brief Motion model that predicts the pose of the robot given the velocities
 * with a constant velocity model
 */
class ConstantVelocityModel : public V2PMotionModel {
public:
  explicit ConstantVelocityModel();

  /**
   * @brief Construct a new ConstantVelocityModel object with a base process noise
   *
   * @param base_process_noise standard non variating noise if used
   */
  explicit ConstantVelocityModel(Eigen::SparseMatrix<double> base_process_noise);

  /**
   * @brief Predict the pose of the robot given the velocities
   *
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d get_next_pose(const Eigen::SparseMatrix<double> &previous_pose,
                                const Eigen::SparseMatrix<double> &velocities,
                                const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::Matrix3d get_jacobian(const Eigen::SparseMatrix<double> &previous_pose,
                               const Eigen::SparseMatrix<double> &velocities,
                               const double delta_t) override;
};