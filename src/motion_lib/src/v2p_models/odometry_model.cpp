
#include "motion_lib/v2p_models/odometry_model.hpp"

#include "common_lib/maths/transformations.hpp"

Eigen::Vector3d OdometryModel::get_pose_difference(const Eigen::Vector3d &previous_pose,
                                                   const Eigen::VectorXd &motion_data,
                                                   [[maybe_unused]] const double delta_t) {
  Eigen::Vector3d pose_difference;
  pose_difference(0) = motion_data(0) - previous_pose(0);
  pose_difference(1) = motion_data(1) - previous_pose(1);
  pose_difference(2) = common_lib::maths::normalize_angle(motion_data(2) - previous_pose(2));
  return pose_difference;
}

Eigen::Matrix3d OdometryModel::get_jacobian_pose(
    [[maybe_unused]] const Eigen::Vector3d &previous_pose,
    [[maybe_unused]] const Eigen::VectorXd &motion_data, [[maybe_unused]] const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  return jacobian;
}

Eigen::MatrixXd OdometryModel::get_jacobian_motion_data(
    [[maybe_unused]] const Eigen::Vector3d &previous_pose,
    [[maybe_unused]] const Eigen::VectorXd &motion_data, [[maybe_unused]] const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  return jacobian;
}