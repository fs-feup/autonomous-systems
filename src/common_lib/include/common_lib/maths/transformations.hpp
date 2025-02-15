#pragma once

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
}  // namespace common_lib::maths