
#include "motion_lib/v2p_models/constant_velocity_turnrate_model.hpp"

#include "common_lib/maths/transformations.hpp"

Eigen::Vector3d ConstantVelocityTurnrateModel::get_pose_difference(
    const Eigen::Vector3d &previous_pose, const Eigen::VectorXd &motion_data,
    const double delta_t) {
  Eigen::Vector3d pose_difference;
  if (::abs(motion_data(2)) < 0.0001) {  // Avoid division by zero when car is going straight
    pose_difference(0) = motion_data(0) * ::cos(previous_pose(2)) * delta_t;
    pose_difference(1) = motion_data(0) * ::sin(previous_pose(2)) * delta_t;
  } else {
    pose_difference(0) =
        motion_data(0) / motion_data(2) *
        (::sin(motion_data(2) * delta_t + previous_pose(2)) - ::sin(previous_pose(2)));
    pose_difference(1) =
        motion_data(0) / motion_data(2) *
        (-::cos(motion_data(2) * delta_t + previous_pose(2)) + ::cos(previous_pose(2)));
  }
  pose_difference(2) = common_lib::maths::normalize_angle(motion_data(2) * delta_t);
  return pose_difference;
}

Eigen::Matrix3d ConstantVelocityTurnrateModel::get_jacobian_pose(
    const Eigen::Vector3d &previous_pose, const Eigen::VectorXd &motion_data,
    const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  if (::abs(motion_data(2)) < 0.001) {  // Avoid division by zero when car is going straight
    jacobian(0, 2) = -motion_data(0) * ::sin(previous_pose(2)) * delta_t;
    jacobian(1, 2) = motion_data(0) * ::cos(previous_pose(2)) * delta_t;
  } else {
    jacobian(0, 2) = motion_data(0) / motion_data(2) *
                     (::cos(motion_data(2) * delta_t + previous_pose(2)) - ::cos(previous_pose(2)));
    jacobian(1, 2) = motion_data(0) / motion_data(2) *
                     (::sin(motion_data(2) * delta_t + previous_pose(2)) - ::sin(previous_pose(2)));
  }
  return jacobian;
}

Eigen::MatrixXd ConstantVelocityTurnrateModel::get_jacobian_motion_data(
    const Eigen::Vector3d &previous_pose, const Eigen::VectorXd &motion_data,
    const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  if (::abs(motion_data(2)) < 0.001) {
    jacobian(0, 0) = ::cos(previous_pose(2)) * delta_t;
    jacobian(1, 0) = ::sin(previous_pose(2)) * delta_t;
  } else {
    jacobian(0, 0) = 1 / motion_data(2) *
                     (::sin(motion_data(2) * delta_t + previous_pose(2)) - ::sin(previous_pose(2)));
    jacobian(0, 2) =
        -(motion_data(0) *
          (::sin(motion_data(2) * delta_t + previous_pose(2)) -
           motion_data(2) * delta_t * ::cos(motion_data(2) * delta_t + previous_pose(2)) -
           ::sin(previous_pose(2))) /
          (motion_data(2) * motion_data(2)));
    jacobian(1, 0) =
        1 / motion_data(2) *
        (-::cos(motion_data(2) * delta_t + previous_pose(2)) + ::cos(previous_pose(2)));
    jacobian(1, 2) =
        (motion_data(0) *
         (::cos(motion_data(2) * delta_t + previous_pose(2)) +
          motion_data(2) * delta_t * ::sin(motion_data(2) * delta_t + previous_pose(2)) -
          ::cos(previous_pose(2))) /
         (motion_data(2) * motion_data(2)));
  }
  jacobian(2, 2) = delta_t;
  return jacobian;
}