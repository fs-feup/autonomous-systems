#pragma once

#include <Eigen/Dense>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

/**
 * @brief Motion model that predicts the pose of the robot given the velocities
 * with a constant velocity model
 */
class ConstantVelocityModel : public V2PMotionModel {
public:
  /**
   * @brief Gives the increments to the pose instead of the next pose
   *
   * @param previous_pose
   * @param motion_data (vx, vy, omega)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d get_pose_difference(const Eigen::Vector3d &previous_pose,
                                      const Eigen::VectorXd &motion_data,
                                      const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to the pose (state)
   * @details This is used to multiplty by the previous covariance to get the matrix, propagating
   * the previous pose error to the current pose error (variance)
   * @param previous_pose
   * @param motion_data
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::Matrix3d get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                    const Eigen::VectorXd &motion_data,
                                    const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to motion data (commands)
   * @details This is used to multiplty by the motion data noise to get the matrix to be summed to
   * the covariance
   * @param previous_pose
   * @param motion_data
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::MatrixXd get_jacobian_motion_data(const Eigen::Vector3d &previous_pose,
                                           [[maybe_unused]] const Eigen::VectorXd &motion_data,
                                           const double delta_t) override;
};