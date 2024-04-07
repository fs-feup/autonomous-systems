#ifndef SRC_SPEED_EST_INCLUDE_UTILS_FORMULAS_HPP_
#define SRC_SPEED_EST_INCLUDE_UTILS_FORMULAS_HPP_

/**
 * @brief Function to keep angle between 0 and 2Pi radians
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

#endif  // SRC_SPEED_EST_INCLUDE_UTILS_FORMULAS_HPP_