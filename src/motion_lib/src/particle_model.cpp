#include "motion_lib/particle_model.hpp"

namespace motion_lib::particle_model {

void update_velocities(Eigen::Vector3f& velocities, double ax, double ay, double angular_velocity,
                       double time_interval) {
  velocities(0) += ax * time_interval;
  velocities(1) += ay * time_interval;
  velocities(2) += angular_velocity;
}

Eigen::Matrix3f jacobian_of_velocity_update() { return Eigen::Matrix3f::Identity(); }

void update_pose(Eigen::VectorXd& state, double vx, double vy, double angular_velocity,
                 double time_interval) {
  state(0) += vx * time_interval;
  state(1) += vy * time_interval;
  state(2) += angular_velocity * time_interval;
}

Eigen::MatrixXd jacobian_of_pose_update(int state_size) {
  return Eigen::MatrixXd::Identity(state_size, state_size);
}
}  // namespace motion_lib::particle_model