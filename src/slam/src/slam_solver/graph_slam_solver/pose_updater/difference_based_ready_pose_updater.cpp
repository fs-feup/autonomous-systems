#include "slam_solver/graph_slam_solver/pose_updater/difference_based_ready_pose_updater.hpp"

DifferenceBasedReadyPoseUpdater::DifferenceBasedReadyPoseUpdater(const SLAMParameters& params)
    : PoseUpdater(params) {
  this->minimum_pose_difference_ = params.slam_min_pose_difference_;
}

DifferenceBasedReadyPoseUpdater::DifferenceBasedReadyPoseUpdater(
    const DifferenceBasedReadyPoseUpdater& other)
    : PoseUpdater(other) {
  this->minimum_pose_difference_ = other.minimum_pose_difference_;
}

DifferenceBasedReadyPoseUpdater& DifferenceBasedReadyPoseUpdater::operator=(
    const DifferenceBasedReadyPoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  PoseUpdater::operator=(other);  // Call base class assignment operator
  this->minimum_pose_difference_ = other.minimum_pose_difference_;

  return *this;
}

bool DifferenceBasedReadyPoseUpdater::pose_ready_for_graph_update() const {
  double pose_difference_norm = ::sqrt(pow(this->_accumulated_pose_difference_(0), 2) +
                                       pow(this->_accumulated_pose_difference_(1), 2) +
                                       pow(this->_accumulated_pose_difference_(2), 2));
  return pose_difference_norm >= this->minimum_pose_difference_;
}
