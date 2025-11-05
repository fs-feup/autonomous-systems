#include "slam_solver/graph_slam_solver/pose_updater/double_pose_updater.hpp"

DoublePoseUpdater::DoublePoseUpdater(const SLAMParameters& params)
    : DifferenceBasedReadyPoseUpdater(params) {
  this->_received_first_velocities_ = false;
  this->_received_first_odometry_ = false;
  this->_accumulated_odometry_pose_difference_ = Eigen::Vector3d::Zero();
  this->_last_odometry_pose_ = Eigen::Vector3d::Zero();
  this->_last_velocities_receive_time_ =
      rclcpp::Time(0);  // Initialize the last velocities receive time
}

DoublePoseUpdater::DoublePoseUpdater(const DoublePoseUpdater& other)
    : DifferenceBasedReadyPoseUpdater(other) {
  this->_received_first_velocities_ = other._received_first_velocities_;
  this->_received_first_odometry_ = other._received_first_odometry_;
  this->_last_velocities_receive_time_ = other._last_velocities_receive_time_;
  this->_accumulated_odometry_pose_difference_ = other._accumulated_odometry_pose_difference_;
  this->_last_odometry_pose_ = other._last_odometry_pose_;
}

DoublePoseUpdater& DoublePoseUpdater::operator=(const DoublePoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  DifferenceBasedReadyPoseUpdater::operator=(other);  // Call base class assignment operator
  this->_received_first_velocities_ = other._received_first_velocities_;
  this->_received_first_odometry_ = other._received_first_odometry_;
  this->_last_velocities_receive_time_ = other._last_velocities_receive_time_;
  this->_accumulated_odometry_pose_difference_ = other._accumulated_odometry_pose_difference_;
  this->_last_odometry_pose_ = other._last_odometry_pose_;

  return *this;
}

void DoublePoseUpdater::predict_pose(const MotionData& motion_data,
                                     std::shared_ptr<V2PMotionModel> motion_model) {}

bool DoublePoseUpdater::second_pose_difference_ready() const {
  return true;  // TODO: implement this shit
}

Eigen::Vector3d DoublePoseUpdater::get_second_accumulated_pose_difference() const {
  return _accumulated_odometry_pose_difference_;
}