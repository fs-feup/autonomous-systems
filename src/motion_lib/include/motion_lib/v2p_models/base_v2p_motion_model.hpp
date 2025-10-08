#pragma once

#include <Eigen/Dense>

/**
 * @brief Base class for the 'vehicle to pose' (more in interal background review) motion models
 *
 * @details This class defines the interface for the motion models that predict the pose of the
 * robot given the velocities
 */
class V2PMotionModel {
  Eigen::Vector3d
      _base_process_noise_;  //< Equivalent to the diagonal of the process noise covariance matrix

public:
  explicit V2PMotionModel();

  /**
   * @brief Construct a new ConstantVelocityModel object with a base process noise
   *
   * @param base_process_noise standard non variating noise if used
   */
  explicit V2PMotionModel(const Eigen::Vector3d base_process_noise);

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
                                        const Eigen::Vector3d &velocities, const double delta_t);

  /**
   * @brief Gives the increments to the pose instead of the next pose
   *
   * @param previous_pose
   * @param velocities (vx, vy, omega)
   * @param delta_t
   * @return Eigen::Vector3d
   */
  virtual Eigen::Vector3d get_pose_difference(const Eigen::Vector3d &previous_pose,
                                              const Eigen::Vector3d &velocities,
                                              const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to the pose (state)
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  virtual Eigen::Matrix3d get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                            const Eigen::Vector3d &velocities,
                                            const double delta_t) = 0;

  /**
   * @brief Get the Jacobian matrix of the motion model in relation to velocities (commands)
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3d
   */
  virtual Eigen::Matrix3d get_jacobian_velocities(
      const Eigen::Vector3d &previous_pose, [[maybe_unused]] const Eigen::Vector3d &velocities,
      const double delta_t) = 0;

  /**
   * @brief Get the process noise matrix
   */
  Eigen::Matrix3d get_base_process_noise_matrix();

  /**
   * @brief Get the process noise vector
   */
  Eigen::Vector3d get_base_process_noise();
};