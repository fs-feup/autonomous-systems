#include "slam_solver/graph_slam_solver/pose_updater.hpp"

std::pair<Eigen::Vector3d, Eigen::Vector3d> PoseUpdater::update_pose(
    const MotionData& motion_data, std::shared_ptr<V2PMotionModel> motion_model) {
  if (!this->_received_first_velocities_) {
    this->_last_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->_last_pose_update_ = motion_data.timestamp_;
    this->_received_first_velocities_ = true;
    return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  }

  double delta = (motion_data.timestamp_ - this->_last_pose_update_).seconds() +
                 (motion_data.timestamp_ - this->_last_pose_update_).nanoseconds() / 1000000000;
  Eigen::Vector3d pose_difference =
      motion_model->get_pose_difference(this->_last_pose_, *(motion_data.velocities_), delta);

  this->_last_pose_ += pose_difference;
  this->_last_pose_update_ = motion_data.timestamp_;

  return {pose_difference, this->_last_pose_};
}

PoseUpdater::PoseUpdater(const PoseUpdater& other) {
  this->_last_pose_ = other._last_pose_;
  this->_last_pose_update_ = other._last_pose_update_;
  this->_received_first_velocities_ = other._received_first_velocities_;
}

PoseUpdater& PoseUpdater::operator=(const PoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  this->_last_pose_ = other._last_pose_;
  this->_last_pose_update_ = other._last_pose_update_;
  this->_received_first_velocities_ = other._received_first_velocities_;

  return *this;
}