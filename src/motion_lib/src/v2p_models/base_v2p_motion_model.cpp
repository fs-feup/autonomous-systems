#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

#include "common_lib/maths/transformations.hpp"

V2PMotionModel::V2PMotionModel(const Eigen::Vector3d base_process_noise)
    : _base_process_noise_(base_process_noise) {}

Eigen::Vector3d V2PMotionModel::get_next_pose(const Eigen::Vector3d &previous_pose,
                                              const Eigen::Vector3d &velocities,
                                              const double delta_t) {
  Eigen::Vector3d next_pose =
      previous_pose + this->get_pose_difference(previous_pose, velocities, delta_t);
  next_pose(2) = common_lib::maths::normalize_angle(next_pose(2));
  return next_pose;
}

V2PMotionModel::V2PMotionModel() : _base_process_noise_(Eigen::Vector3d::Zero()) {}

Eigen::Matrix3d V2PMotionModel::get_base_process_noise_matrix() {
  return _base_process_noise_.asDiagonal();
}

Eigen::Vector3d V2PMotionModel::get_base_process_noise() { return _base_process_noise_; }