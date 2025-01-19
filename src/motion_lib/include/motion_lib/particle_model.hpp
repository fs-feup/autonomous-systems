#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/structures/velocities.hpp"

namespace motion_lib::particle_model {
/**
 * @brief Update velocities assuming period of time with constant accelerations acting in the Center
 * of Mass
 *
 * @param velocities size 3 vector with {velocity along x axis; velocity along y axis; angular
 * velocity around z axis}
 * @param ax acceleration along x axis
 * @param ay acceleration along y axis
 * @param angular_velocity angular velocity around z axis
 * @param time_interval time interval over which the update is applied
 */
void update_velocities(Eigen::Vector3f& velocities, double ax, double ay, double angular_velocity,
                       double time_interval);
/**
 * @brief Compute the Jacobian matrix of the velocity update function
 */
Eigen::Matrix3f jacobian_of_velocity_update();

void update_pose(Eigen::VectorXd& state, double vx, double vy, double angular_velocity,
                 double time_interval);

Eigen::MatrixXd jacobian_of_pose_update(int state_size);

}  // namespace motion_lib::particle_model