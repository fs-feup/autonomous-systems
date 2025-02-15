#pragma once

#include <Eigen/Dense>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

class ConstantVelocityModel : public BaseV2PMotionModel {
  double calculate_velocity_modulus(const Eigen::Vector3f &velocities) const;

public:
  explicit ConstantVelocityModel();
  explicit ConstantVelocityModel(Eigen::Vector3f base_process_noise);

  /**
   * @brief Predict the pose of the robot given the velocities
   *
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Vector3f
   */
  Eigen::Vector3f get_next_pose(const Eigen::Vector3f &previous_pose,
                                const Eigen::Vector3f &velocities, const double delta_t) override;

  /**
   * @brief Get the Jacobian matrix of the motion model
   * @param previous_pose
   * @param velocities
   * @param delta_t
   * @return Eigen::Matrix3f
   */
  Eigen::Matrix3f get_jacobian(const Eigen::Vector3f &previous_pose,
                               const Eigen::Vector3f &velocities, const double delta_t) override;
};