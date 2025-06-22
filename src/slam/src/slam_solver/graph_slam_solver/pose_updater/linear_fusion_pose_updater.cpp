#include "slam_solver/graph_slam_solver/pose_updater/linear_fusion_pose_updater.hpp"

void LinearFusionPoseUpdater::predict_pose(const MotionData& motion_data,
                                           std::shared_ptr<V2PMotionModel> motion_model) {
  Eigen::Vector3d pose_difference = Eigen::Vector3d::Zero();
  Eigen::Vector3d next_pose = Eigen::Vector3d::Zero();

  if (motion_data.type_ == MotionInputType::VELOCITIES) {
    if (!this->_received_first_velocities_) {
      this->_last_velocities_pose_update_ = motion_data.timestamp_;
      this->_received_first_velocities_ = true;
      return;
    }
    double delta =
        (motion_data.timestamp_ - this->_last_velocities_pose_update_).seconds() +
        (motion_data.timestamp_ - this->_last_velocities_pose_update_).nanoseconds() / 1000000000;
    this->_last_velocities_pose_update_ = motion_data.timestamp_;
    pose_difference = motion_model->get_pose_difference(this->_second_to_last_pose_,
                                                        *(motion_data.motion_data_), delta);
    this->_accumulated_pose_difference_ += pose_difference;
    this->_last_pose_ = motion_model->get_next_pose(this->_second_to_last_pose_,
                                                    *(motion_data.motion_data_), delta);
    this->_waiting_to_calculate_average_ = true;
  } else if (motion_data.type_ == MotionInputType::ODOMETRY) {
    if (!this->_received_first_odometry_) {
      this->_last_odometry_pose_ = Eigen::Vector3d::Zero();
      this->_received_first_odometry_ = true;
      return;
    }
    double delta = (motion_data.timestamp_ - this->_last_pose_update_).seconds() +
                   (motion_data.timestamp_ - this->_last_pose_update_).nanoseconds() / 1000000000;
    pose_difference = motion_model->get_pose_difference(this->_last_odometry_pose_,
                                                        *(motion_data.motion_data_), delta);
    this->_accumulated_odometry_pose_difference_ += pose_difference;
    this->_last_odometry_pose_ = *(motion_data.motion_data_);
    next_pose = motion_model->get_next_pose(this->_second_to_last_pose_,
                                            *(motion_data.motion_data_), delta);
    // this->_last_pose = this->_last_pose_ + next_pose / 2;
    this->_last_pose_update_ = motion_data.timestamp_;
    this->_second_to_last_pose_ = this->_last_pose_;
    this->_waiting_to_calculate_average_ = false;
  }
}

LinearFusionPoseUpdater::LinearFusionPoseUpdater(const LinearFusionPoseUpdater& other)
    : PoseUpdater(other) {
  this->_received_first_velocities_ = other._received_first_velocities_;
}

LinearFusionPoseUpdater& LinearFusionPoseUpdater::operator=(const LinearFusionPoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  PoseUpdater::operator=(other);  // Call base class assignment operator
  this->_received_first_velocities_ = other._received_first_velocities_;

  return *this;
}
