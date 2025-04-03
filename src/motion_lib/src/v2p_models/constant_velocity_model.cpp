
#include "motion_lib/v2p_models/constant_velocity_model.hpp"

#include "common_lib/maths/transformations.hpp"

ConstantVelocityModel::ConstantVelocityModel(const Eigen::Vector3d base_process_noise)
    : V2PMotionModel(base_process_noise) {}

ConstantVelocityModel::ConstantVelocityModel() : V2PMotionModel() {}

Eigen::Vector3d ConstantVelocityModel::get_next_pose(const Eigen::Vector3d &previous_pose,
                                                     const Eigen::Vector3d &velocities,
                                                     const double delta_t) {
  Eigen::Vector3d next_pose;
  next_pose(0) =
      previous_pose(0) +
      (velocities(0) * ::cos(previous_pose(2)) - velocities(1) * ::sin(previous_pose(2))) * delta_t;
  next_pose(1) =
      previous_pose(1) +
      (velocities(0) * ::sin(previous_pose(2)) + velocities(1) * ::cos(previous_pose(2))) * delta_t;
  next_pose(2) = common_lib::maths::normalize_angle(previous_pose(2) + velocities(2) * delta_t);
  return next_pose;
}

Eigen::Matrix3d ConstantVelocityModel::get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                                         const Eigen::Vector3d &velocities,
                                                         const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  jacobian(0, 2) =
      -(velocities(0) * ::sin(previous_pose(2)) + velocities(1) * ::cos(previous_pose(2))) *
      delta_t;
  jacobian(1, 2) =
      (velocities(0) * ::cos(previous_pose(2)) - velocities(1) * ::sin(previous_pose(2))) * delta_t;
  return jacobian;
}

Eigen::Matrix3d ConstantVelocityModel::get_jacobian_velocities(const Eigen::Vector3d &previous_pose,
                                                               const Eigen::Vector3d &velocities,
                                                               const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  jacobian(0, 0) = ::cos(previous_pose(2)) * delta_t;
  jacobian(0, 1) = -::sin(previous_pose(2)) * delta_t;
  jacobian(1, 0) = ::sin(previous_pose(2)) * delta_t;
  jacobian(1, 1) = ::cos(previous_pose(2)) * delta_t;
  jacobian(2, 2) = delta_t;
  return jacobian;
}