#pragma once

#include <Eigen/Dense>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

/**
 * @brief Motion model that predicts the pose of the robot given the velocities
 * with a constant velocity model and turnrate model
 */
class ConstantVelocityTurnrateModel : public V2PMotionModel {
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
   * @brief Get the Jacobian matrix of the motion model in relation to motion_data (commands)
   * @param previous_pose
   * @param motion_data
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::Matrix3d get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                    const Eigen::VectorXd &motion_data,
                                    const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to motion_data (commands)
   * @param previous_pose
   * @param motion_data
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::MatrixXd get_jacobian_motion_data(const Eigen::Vector3d &previous_pose,
                                           const Eigen::VectorXd &motion_data,
                                           const double delta_t) override;
};