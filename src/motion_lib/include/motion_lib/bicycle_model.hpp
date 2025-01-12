#pragma once
#include <Eigen/Dense>
#include <utility>

#include "common_lib/structures/position.hpp"

namespace motion_lib::bicycle_model {
/**
 * @brief Converts the odometry data to translational and rotational
 * velocities
 *
 * @param rl_rpm rear left wheel rpms
 * @param fl_rpm front left wheel rpms
 * @param rr_rpm rear right wheel rpms
 * @param fr_rpm front right wheel rpms
 * @param steering_angle steering angle in radians
 *
 * @return std::pair<double, double> translational velocity, rotational velocity
 */
std::pair<double, double> odometry_to_velocities_transform(double rl_rpm,
                                                           [[maybe_unused]] double fl_rpm,
                                                           double rr_rpm,
                                                           [[maybe_unused]] double fr_rpm,
                                                           double steering_angle);

/**
 * @brief Calculate rear axis coordinates
 *
 * @param cg
 * @param orientation
 * @param dist_cg_2_rear_axis
 *
 * @return Point
 */
common_lib::structures::Position cg_2_rear_axis(common_lib::structures::Position cg,
                                                double orientation, double dist_cg_2_rear_axis);

/**
 * @brief Estimate observations assuming a bicycle model and a set of parameters
 *
 * @param state vector of velocities {velocity_x, velocity_y, rotational_velocity}
 * @param wheel_base distance between the front and rear wheels
 * @param weight_distribution_front percentage of the vehicle's weight on the front wheels
 * @param gear_ratio rotations of the motor for each rotation of the rear wheels
 * @param wheel_radius radius of the rear wheels
 * @return Eigen::VectorXd vector of observations {fl_rpm, fr_rpm, rl_rpm, rr_rpm, steering_angle,
 * motor_rpm}
 */
Eigen::VectorXd estimate_observations(Eigen::Vector3d& state, double wheel_base,
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
 * @return Eigen::MatrixXd jacobian matrix of the function estimate_observations (6x3)
 */
Eigen::MatrixXd jacobian_of_observation_estimation(Eigen::Vector3d& state, double wheel_base,
                                                   double weight_distribution_front,
                                                   double gear_ratio, double wheel_radius);
}  // namespace motion_lib::bicycle_model