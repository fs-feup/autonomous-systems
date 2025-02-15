
#include "motion_lib/v2p_models/constant_velocity_model.hpp"

#include "common_lib/maths/transformations.hpp"

ConstantVelocityModel::ConstantVelocityModel(Eigen::Vector3f base_process_noise)
    : BaseV2PMotionModel(base_process_noise) {}

ConstantVelocityModel::ConstantVelocityModel() : BaseV2PMotionModel() {}

Eigen::Vector3f ConstantVelocityModel::get_next_pose(const Eigen::Vector3f &previous_pose,
                                                     const Eigen::Vector3f &velocities,
                                                     const double delta_t) {
  Eigen::Vector3f next_pose;
  double velocity_modulus = this->calculate_velocity_modulus(velocities);
  next_pose(2) = common_lib::maths::normalize_angle(previous_pose(2) + velocities(2) * delta_t);
  next_pose(0) = previous_pose(0) + velocity_modulus * cos(next_pose(2)) * delta_t;
  next_pose(1) = previous_pose(1) + velocity_modulus * sin(next_pose(2)) * delta_t;
  return next_pose;
}

Eigen::Matrix3f ConstantVelocityModel::get_jacobian(const Eigen::Vector3f &previous_pose,
                                                    const Eigen::Vector3f &velocities,
                                                    const double delta_t) {
  Eigen::Matrix3f jacobian = Eigen::Matrix3f::Identity();
  double velocity_modulus = this->calculate_velocity_modulus(velocities);
  jacobian(0, 2) = -velocity_modulus * sin(previous_pose(2) + velocities(2) * delta_t) * delta_t;
  jacobian(1, 2) = velocity_modulus * cos(previous_pose(2) + velocities(2) * delta_t) * delta_t;
  return jacobian;
}

double ConstantVelocityModel::calculate_velocity_modulus(const Eigen::Vector3f &velocities) const {
  return sqrt(pow(velocities(0), 2) + pow(velocities(1), 2));
}