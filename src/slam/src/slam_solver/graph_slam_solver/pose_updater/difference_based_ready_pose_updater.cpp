#include "slam_solver/graph_slam_solver/pose_updater/difference_based_ready_pose_updater.hpp"

#include "slam_solver/graph_slam_solver/utils.hpp"

DifferenceBasedReadyPoseUpdater::DifferenceBasedReadyPoseUpdater(const SLAMParameters& params)
    : PoseUpdater(params) {
  this->minimum_pose_difference_ = params.slam_min_pose_difference_;
}

DifferenceBasedReadyPoseUpdater::DifferenceBasedReadyPoseUpdater(
    const DifferenceBasedReadyPoseUpdater& other)
    : PoseUpdater(other) {
  this->minimum_pose_difference_ = other.minimum_pose_difference_;
}

DifferenceBasedReadyPoseUpdater::~DifferenceBasedReadyPoseUpdater() = default;

DifferenceBasedReadyPoseUpdater& DifferenceBasedReadyPoseUpdater::operator=(
    const DifferenceBasedReadyPoseUpdater& other) {
  if (this == &other) {
    return *this;  // Prevent self-assignment
  }

  // Copy each member individually
  PoseUpdater::operator=(other);  // Call base class assignment operator
  this->minimum_pose_difference_ = other.minimum_pose_difference_;

  return *this;
}

std::shared_ptr<PoseUpdater> DifferenceBasedReadyPoseUpdater::clone() const {
  return std::make_shared<DifferenceBasedReadyPoseUpdater>(*this);
}

bool DifferenceBasedReadyPoseUpdater::pose_ready_for_graph_update() const {
  Eigen::Vector3d pose_difference =
      pose_difference_eigen(this->_last_graphed_pose_, this->_last_pose_);
  double pose_difference_norm =
      ::sqrt(pow(pose_difference(0), 2) + pow(pose_difference(1), 2) + pow(pose_difference(2), 2));
  return pose_difference_norm >= this->minimum_pose_difference_;
}
