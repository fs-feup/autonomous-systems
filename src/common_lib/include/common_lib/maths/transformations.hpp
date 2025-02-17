#pragma once
#include <Eigen/Dense>
namespace common_lib::maths {
/**
 * @brief Function to keep angle in [-Pi, Pi[ radians
 *
 * @param angle
 * @return double
 */
double normalize_angle(double angle);

/**
 * @brief Function to get the translational
 * velocity of a wheel from rpm and wheel diameter
 *
 */
double get_wheel_velocity_from_rpm(const double rpm, const double wheel_diameter);

/**
 * @brief Function to get the rotation matrix
 * for a given angle
 *
 * @param angle
 * @return Eigen::Matrix2d
 */
Eigen::Matrix2d get_rotation_matrix(const double angle);
}  // namespace common_lib::maths