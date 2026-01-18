#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

#include "common_lib/maths/transformations.hpp"

Eigen::Vector3d V2PMotionModel::get_next_pose(
    const Eigen::Vector3d &previous_pose, const Eigen::VectorXd &motion_data,
    const double delta_t) {  // TODO: change to XD and create new acceleration model
  Eigen::Vector3d next_pose =
      previous_pose + this->get_pose_difference(previous_pose, motion_data, delta_t);
  next_pose(2) = common_lib::maths::normalize_angle(next_pose(2));
  return next_pose;
}
