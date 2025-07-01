#pragma once

#include <Eigen/Dense>

/**
 * @brief Base class for the 'vehicle to pose' (more in interal background review) motion models
 *
 * @details This class defines the interface for the motion models that predict the pose of the
 * robot given the velocities
 */
class V2PMotionModel {
public:

  virtual ~V2PMotionModel() = default;

  /**
   * @brief Predict the pose of the robot given the velocities
   *
   * @param previous_pose
   * @param velocities (vx, vy, omega)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  virtual Eigen::Vector3d get_next_pose(const Eigen::Vector3d &previous_pose,
                                        const Eigen::VectorXd &motion_data, const double delta_t);

  /**
   * @brief Gives the increments to the pose instead of the next pose
   *
   * @param previous_pose
   * @param velocities (vx, vy, omega)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  virtual Eigen::Vector3d get_pose_difference(const Eigen::Vector3d &previous_pose,
                                              const Eigen::VectorXd &motion_data,
                                              const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to the pose (state)
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  virtual Eigen::Matrix3d get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                            const Eigen::VectorXd &motion_data,
                                            const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to velocities (commands)
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  virtual Eigen::MatrixXd get_jacobian_motion_data(
      const Eigen::Vector3d &previous_pose, [[maybe_unused]] const Eigen::VectorXd &motion_data,
      const double delta_t) = 0;
};