#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"

PoseUpdater::PoseUpdater(const PoseUpdater& other) {
  this->_last_pose_ = other._last_pose_;
  this->_last_pose_update_ = other._last_pose_update_;
}

PoseUpdater& PoseUpdater::operator=(const PoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  this->_last_pose_ = other._last_pose_;
  this->_last_pose_update_ = other._last_pose_update_;

  return *this;
}

void PoseUpdater::update_pose(const Eigen::Vector3d& last_pose) {
  this->_last_pose_ = last_pose;
  this->_last_pose_update_ = rclcpp::Clock().now();
  this->_accumulated_pose_difference_ =
      Eigen::Vector3d::Zero();  // Reset accumulated pose difference
}
