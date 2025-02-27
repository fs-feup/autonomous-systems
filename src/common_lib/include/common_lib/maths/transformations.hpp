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

/**
 * @brief Transform points from a global reference frame to a local reference frame
 *
 * @param local_reference_frame coordinates of the local reference frame in the global reference
 * frame [x, y, rotation angle]
 * @param global_points points in the global reference frame in format [x1, y1, x2, y2, ...]
 * @return Eigen::VectorXd points in the local reference frame in format [x1, y1, x2, y2, ...]
 */
Eigen::VectorXd global_to_local_referential(const Eigen::Vector3d& local_reference_frame,
                                            const Eigen::VectorXd& global_points);

/**
 * @brief Transform points from a local reference frame to a global reference frame
 *
 * @param local_reference_frame coordinates of the local reference frame in the global reference
 * frame [x, y, rotation angle]
 * @param local_points points in the local reference frame in format [x1, y1, x2, y2, ...]
 * @return Eigen::VectorXd points in the global reference frame in format [x1, y1, x2, y2, ...]
 */
Eigen::VectorXd local_to_global_referential(const Eigen::Vector3d& local_reference_frame,
                                            const Eigen::VectorXd& local_points);
}  // namespace common_lib::maths