#pragma once
#include <utility>

#include "common_lib/structures/position.hpp"

namespace common_lib::vehicle_dynamics {
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

}  // namespace common_lib::vehicle_dynamics