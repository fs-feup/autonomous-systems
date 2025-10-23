#pragma once

#pragma once

#include <Eigen/Dense>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

/**
 * @brief Motion model to apply to odometry sources which already output pose
 *
 * @details This model is used so that odometry sources also have a motion model and code remains
 * the same for every solution
 */
class OdometryModel : public V2PMotionModel {
public:
  /**
   * @brief Gives the increments to the pose instead of the next pose
   *
   * @param previous_pose
   * @param motion_data (Deltax, Deltay, Delta_theta)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d get_pose_difference(const Eigen::Vector3d &previous_pose,
                                      const Eigen::VectorXd &motion_data,
                                      const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to motion_data (commands)
   * @param previous_pose
   * @param motion_data (Deltax, Deltay, Delta_theta)
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
   * @param motion_data (Deltax, Deltay, Delta_theta)
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::MatrixXd get_jacobian_motion_data(const Eigen::Vector3d &previous_pose,
                                           [[maybe_unused]] const Eigen::VectorXd &motion_data,
                                           const double delta_t) override;
};