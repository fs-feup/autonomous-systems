#include "slam_solver/graph_slam_solver/pose_updater/odometry_based_pose_updater.hpp"

void OdometryBasedPoseUpdater::predict_pose(const MotionData& motion_data,
                                            std::shared_ptr<V2PMotionModel> motion_model) {
  if (!this->_received_first_odometry_) {
    this->_last_odometry_pose_ = Eigen::Vector3d::Zero();
    this->_received_first_odometry_ = true;
    return;
  }
  double delta = (motion_data.timestamp_ - this->_last_pose_update_).seconds() +
                 (motion_data.timestamp_ - this->_last_pose_update_).nanoseconds() / 1000000000;
  Eigen::Vector3d pose_difference = motion_model->get_pose_difference(
      this->_last_odometry_pose_, *(motion_data.motion_data_), delta);
  this->_accumulated_pose_difference_ += pose_difference;
  this->_last_odometry_pose_ = *(motion_data.motion_data_);
  this->_last_pose_ =
      motion_model->get_next_pose(this->_last_pose_, *(motion_data.motion_data_), delta);
  this->_last_pose_update_ = motion_data.timestamp_;
}

OdometryBasedPoseUpdater::OdometryBasedPoseUpdater(const OdometryBasedPoseUpdater& other)
    : PoseUpdater(other) {
  this->_received_first_odometry_ = other._received_first_odometry_;
}

OdometryBasedPoseUpdater& OdometryBasedPoseUpdater::operator=(
    const OdometryBasedPoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  PoseUpdater::operator=(other);  // Call base class assignment operator
  this->_received_first_odometry_ = other._received_first_odometry_;

  return *this;
}