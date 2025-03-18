
#include "motion_lib/v2p_models/constant_velocity_turnrate_model.hpp"

#include "common_lib/maths/transformations.hpp"

ConstantVelocityTurnrateModel::ConstantVelocityTurnrateModel(Eigen::Vector3d base_process_noise)
    : V2PMotionModel(base_process_noise) {}

ConstantVelocityTurnrateModel::ConstantVelocityTurnrateModel() : V2PMotionModel() {}

Eigen::Vector3d ConstantVelocityTurnrateModel::get_next_pose(const Eigen::Vector3d &previous_pose,
                                                             const Eigen::Vector3d &velocities,
                                                             const double delta_t) {
  Eigen::Vector3d next_pose;
  if (::abs(velocities(2)) < 0.0001) {
    next_pose(0) = previous_pose(0) + (velocities(0) * ::cos(previous_pose(2)) -
                                       velocities(1) * ::sin(previous_pose(2))) *
                                          delta_t;
    next_pose(1) = previous_pose(1) + (velocities(0) * ::sin(previous_pose(2)) +
                                       velocities(1) * ::cos(previous_pose(2))) *
                                          delta_t;
  } else {
    next_pose(0) = previous_pose(0) + velocities(0) / velocities(2) *
                                          (::sin(velocities(2) * delta_t + previous_pose(2)) -
                                           ::sin(previous_pose(2)));
    next_pose(1) = previous_pose(1) + velocities(0) / velocities(2) *
                                          (-::cos(velocities(2) * delta_t + previous_pose(2)) +
                                           ::cos(previous_pose(2)));
  }
  next_pose(2) = common_lib::maths::normalize_angle(previous_pose(2) + velocities(2) * delta_t);
  return next_pose;
}

Eigen::Matrix3d ConstantVelocityTurnrateModel::get_jacobian(const Eigen::Vector3d &previous_pose,
                                                            const Eigen::Vector3d &velocities,
                                                            const double delta_t) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  if (::abs(velocities(2)) < 0.001) {
    jacobian(0, 2) =
        -(velocities(0) * ::sin(previous_pose(2)) + velocities(1) * ::cos(previous_pose(2))) *
        delta_t;
    jacobian(1, 2) =
        (velocities(0) * ::cos(previous_pose(2)) - velocities(1) * ::sin(previous_pose(2))) *
        delta_t;
  } else {
    jacobian(0, 2) = velocities(0) / velocities(2) *
                     (::cos(velocities(2) * delta_t + previous_pose(2)) - ::cos(previous_pose(2)));
    jacobian(1, 2) = velocities(0) / velocities(2) *
                     (::sin(velocities(2) * delta_t + previous_pose(2)) - ::sin(previous_pose(2)));
  }
  return jacobian;
}