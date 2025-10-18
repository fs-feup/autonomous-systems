
#include "motion_lib/v2p_models/constant_velocity_model.hpp"

#include "common_lib/maths/transformations.hpp"

Eigen::Vector3d ConstantVelocityModel::get_pose_difference(const Eigen::Vector3d &previous_pose,
                                                           const Eigen::VectorXd &motion_data,
                                                           const double delta_t) {
  Eigen::Vector3d pose_difference;
  pose_difference(0) =
      (motion_data(0) * ::cos(previous_pose(2)) - motion_data(1) * ::sin(previous_pose(2))) *
      delta_t;
  pose_difference(1) =
      (motion_data(0) * ::sin(previous_pose(2)) + motion_data(1) * ::cos(previous_pose(2))) *
      delta_t;
  pose_difference(2) = common_lib::maths::normalize_angle(motion_data(2) * delta_t);
  return pose_difference;
}

Eigen::Matrix3d ConstantVelocityModel::get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                                         const Eigen::VectorXd &motion_data,
                                                         const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  jacobian(0, 2) =
      -(motion_data(0) * ::sin(previous_pose(2)) + motion_data(1) * ::cos(previous_pose(2))) *
      delta_t;
  jacobian(1, 2) =
      (motion_data(0) * ::cos(previous_pose(2)) - motion_data(1) * ::sin(previous_pose(2))) *
      delta_t;
  return jacobian;
}

Eigen::MatrixXd ConstantVelocityModel::get_jacobian_motion_data(
    const Eigen::Vector3d &previous_pose, [[maybe_unused]] const Eigen::VectorXd &motion_data,
    const double delta_t) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, 4);
  jacobian(0, 0) = ::cos(previous_pose(2)) * delta_t;
  jacobian(0, 1) = -::sin(previous_pose(2)) * delta_t;
  jacobian(1, 0) = ::sin(previous_pose(2)) * delta_t;
  jacobian(1, 1) = ::cos(previous_pose(2)) * delta_t;
  jacobian(2, 2) = delta_t;
  return jacobian;
}