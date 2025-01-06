#pragma once
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <utility>

namespace observation_lib::bicycle_model {
/**
 * @brief Estimate observations assuming a bicycle model and a set of parameters
 *
 * @param state vector of velocities {velocity_x, velocity_y, rotational_velocity}
 * @param wheel_base distance between the front and rear wheels
 * @param weight_distribution_front percentage of the vehicle's weight on the front wheels
 * @param gear_ratio rotations of the motor for each rotation of the rear wheels
 * @param wheel_radius radius of the rear wheels
 * @return Eigen::VectorXf vector of observations {fl_rpm, fr_rpm, rl_rpm, rr_rpm, steering_angle,
 * motor_rpm}
 */
Eigen::VectorXf estimate_observations(Eigen::Vector3f& state, double wheel_base,
                                      double weight_distribution_front, double gear_ratio,
                                      double wheel_radius);

/**
 * @brief jacobian of the function estimate_observations with respect to the state
 *
 * @param state vector of velocities {velocity_x, velocity_y, rotational_velocity}
 * @param wheel_base distance between the front and rear wheels
 * @param weight_distribution_front percentage of the vehicle's weight on the front wheels
 * @param gear_ratio rotations of the motor for each rotation of the rear wheels
 * @param wheel_radius radius of the rear wheels
 * @return Eigen::MatrixXf jacobian matrix of the function estimate_observations (6x3)
 */
Eigen::MatrixXf jacobian_of_observation_estimation(Eigen::Vector3f& state, double wheel_base,
                                                   double weight_distribution_front,
                                                   double gear_ratio, double wheel_radius);

}  // namespace observation_lib::bicycle_model