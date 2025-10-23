#pragma once

#include <Eigen/Dense>

/**
 * @brief Base class for the 'vehicle to pose' (more in interal background review) motion models
 *
 * @details This class defines the interface for the motion models that predict the pose of the
 * robot given the motion data
 */
class V2PMotionModel {
public:
  virtual ~V2PMotionModel() = default;

  /**
   * @brief Predict the pose of the robot given the motion data
   *
   * @param previous_pose
   * @param motion_data (vx, vy, omega, ax)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  virtual Eigen::Vector3d get_next_pose(const Eigen::Vector3d &previous_pose,
                                        const Eigen::VectorXd &motion_data, const double delta_t);

  /**
   * @brief Gives the increments to the pose instead of the next pose
   *
   * @param previous_pose
   * @param motion_data (vx, vy, omega, ax)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  virtual Eigen::Vector3d get_pose_difference(const Eigen::Vector3d &previous_pose,
                                              const Eigen::VectorXd &motion_data,
                                              const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to the pose (state)
   * @details This is used to multiplty by the previous covariance to get the matrix, propagating
   * the previous pose error to the current pose error (variance)
   * @param previous_pose
   * @param motion_data (vx, vy, omega, ax)
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  virtual Eigen::Matrix3d get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                            const Eigen::VectorXd &motion_data,
                                            const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to motion data (commands)
   * @details This is used to multiplty by the motion data noise to get the matrix to be summed to
   * the covariance
   * @param previous_pose
   * @param motion_data (vx, vy, omega, ax)
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  virtual Eigen::MatrixXd get_jacobian_motion_data(
      const Eigen::Vector3d &previous_pose, [[maybe_unused]] const Eigen::VectorXd &motion_data,
      const double delta_t) = 0;
};