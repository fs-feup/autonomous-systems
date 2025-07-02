#include "slam_solver/graph_slam_solver/pose_updater/odometry_based_pose_updater.hpp"

OdometryBasedPoseUpdater::OdometryBasedPoseUpdater(const SLAMParameters& params)
    : DifferenceBasedReadyPoseUpdater(params) {
  this->_received_first_odometry_ = false;
  this->_last_odometry_pose_ = Eigen::Vector3d::Zero();
}

OdometryBasedPoseUpdater::OdometryBasedPoseUpdater(const OdometryBasedPoseUpdater& other)
    : DifferenceBasedReadyPoseUpdater(other) {
  this->_received_first_odometry_ = other._received_first_odometry_;
  this->_last_odometry_pose_ = other._last_odometry_pose_;
}

OdometryBasedPoseUpdater& OdometryBasedPoseUpdater::operator=(
    const OdometryBasedPoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  DifferenceBasedReadyPoseUpdater::operator=(other);  // Call base class assignment operator
  this->_received_first_odometry_ = other._received_first_odometry_;
  this->_last_odometry_pose_ = other._last_odometry_pose_;

  return *this;
}

void OdometryBasedPoseUpdater::predict_pose(const MotionData& motion_data,
                                            std::shared_ptr<V2PMotionModel> motion_model) {
  if (!this->_received_first_odometry_) {
    this->_last_odometry_pose_ = Eigen::Vector3d::Zero();
    this->_received_first_odometry_ = true;
    return;
  }
  Eigen::Vector3d pose_difference = motion_model->get_pose_difference(
      this->_last_odometry_pose_, *(motion_data.motion_data_), 0.0);
  this->_accumulated_pose_difference_ += pose_difference;
  this->_last_odometry_pose_ = *(motion_data.motion_data_);
  this->_last_pose_ =
      motion_model->get_next_pose(this->_last_pose_, *(motion_data.motion_data_), 0.0);
  this->_last_pose_update_ = motion_data.timestamp_;
}