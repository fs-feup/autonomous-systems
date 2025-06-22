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
  explicit OdometryModel();

  /**
   * @brief Construct a new OdometryModel object with a base process noise
   *
   * @param base_process_noise standard non variating noise if used
   */
  explicit OdometryModel(const Eigen::Vector3d base_process_noise);

  /**
   * @brief Gives the increments to the pose instead of the next pose
   *
   * @param previous_pose
   * @param velocities (vx, vy, omega)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d get_pose_difference(const Eigen::Vector3d &previous_pose,
                                      const Eigen::Vector3d &velocities,
                                      const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to velocities (commands)
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::Matrix3d get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                    const Eigen::Vector3d &velocities,
                                    const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to velocities (commands)
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  Eigen::Matrix3d get_jacobian_velocities(const Eigen::Vector3d &previous_pose,
                                          [[maybe_unused]] const Eigen::Vector3d &velocities,
                                          const double delta_t) override;
};