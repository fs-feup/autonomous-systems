
#include "motion_lib/v2p_models/odometry_model.hpp"

#include "common_lib/maths/transformations.hpp"

OdometryModel::OdometryModel(const Eigen::Vector3d base_process_noise)
    : V2PMotionModel(base_process_noise) {}

OdometryModel::OdometryModel() : V2PMotionModel() {}

Eigen::Vector3d OdometryModel::get_pose_difference(const Eigen::Vector3d &previous_pose,
                                                   const Eigen::Vector3d &velocities,
                                                   const double delta_t) {
  Eigen::Vector3d pose_difference;
  pose_difference(0) = velocities(0) - previous_pose(0);
  pose_difference(1) = velocities(1) - previous_pose(1);
  pose_difference(2) = common_lib::maths::normalize_angle(velocities(2) - previous_pose(2));
  return pose_difference;
}

Eigen::Matrix3d OdometryModel::get_jacobian_pose(const Eigen::Vector3d &previous_pose,
                                                 const Eigen::Vector3d &velocities,
                                                 const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  return jacobian;
}

Eigen::Matrix3d OdometryModel::get_jacobian_velocities(
    const Eigen::Vector3d &previous_pose, [[maybe_unused]] const Eigen::Vector3d &velocities,
    const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  return jacobian;
}