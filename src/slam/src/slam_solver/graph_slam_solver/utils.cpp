#include "slam_solver/graph_slam_solver/utils.hpp"

gtsam::Pose2 eigen_to_gtsam_pose(const Eigen::Vector3d& pose) {
  return gtsam::Pose2(pose[0], pose[1], pose[2]);
}

Eigen::Vector3d gtsam_pose_to_eigen(const gtsam::Pose2& pose) {
  return Eigen::Vector3d(pose.x(), pose.y(), pose.theta());
}