#include "slam_solver/graph_slam_solver/pose_updater/velocities_based_pose_updater.hpp"

VelocitiesBasedPoseUpdater::VelocitiesBasedPoseUpdater(const SLAMParameters& params)
    : DifferenceBasedReadyPoseUpdater(params) {
  this->_received_first_velocities_ = false;  // Initialize the flag to false
  this->_last_velocities_receive_time_ =
      rclcpp::Time(0);  // Initialize the last velocities receive time
}

VelocitiesBasedPoseUpdater::VelocitiesBasedPoseUpdater(const VelocitiesBasedPoseUpdater& other)
    : DifferenceBasedReadyPoseUpdater(other) {
  this->_received_first_velocities_ = other._received_first_velocities_;
  this->_last_velocities_receive_time_ = other._last_velocities_receive_time_;
}

VelocitiesBasedPoseUpdater& VelocitiesBasedPoseUpdater::operator=(
    const VelocitiesBasedPoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  DifferenceBasedReadyPoseUpdater::operator=(other);  // Call base class assignment operator
  this->_received_first_velocities_ = other._received_first_velocities_;
  this->_last_velocities_receive_time_ = other._last_velocities_receive_time_;

  return *this;
}

void VelocitiesBasedPoseUpdater::predict_pose(const MotionData& motion_data,
                                              std::shared_ptr<V2PMotionModel> motion_model) {
  if (!this->_received_first_velocities_) {
    this->_last_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->_last_velocities_receive_time_ = motion_data.timestamp_;
    this->_received_first_velocities_ = true;
    return;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "Timestamps - Last: %f, Current: %f",
               this->_last_velocities_receive_time_.seconds(), motion_data.timestamp_.seconds());

  double delta =
      (motion_data.timestamp_ - this->_last_velocities_receive_time_).seconds() +
      (motion_data.timestamp_ - this->_last_velocities_receive_time_).nanoseconds() / 1000000000;

  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "Time delta for pose prediction: %f seconds", delta);
  Eigen::Vector3d pose_difference =
      motion_model->get_pose_difference(this->_last_pose_, *(motion_data.motion_data_), delta);
  this->_accumulated_pose_difference_ += pose_difference;
  this->_last_pose_ =
      motion_model->get_next_pose(this->_last_pose_, *(motion_data.motion_data_), delta);
  this->_last_velocities_receive_time_ = motion_data.timestamp_;
  this->_last_pose_update_ = motion_data.timestamp_;
}