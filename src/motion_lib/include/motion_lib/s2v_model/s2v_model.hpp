#pragma once
#include <Eigen/Dense>
#include <utility>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/position.hpp"
/**
 * @brief Interface for models that convert data from sensor reference frame to vehicle reference
 * frame and vice versa
 *
 */
class S2VModel {
public:
  /**
   * @brief Calculates the translational and rotational velocities of the vehicle from wheel
   * rotations and steering angle
   *
   * @param rl_rpm rear left wheel rpms
   * @param fl_rpm front left wheel rpms
   * @param rr_rpm rear right wheel rpms
   * @param fr_rpm front right wheel rpms
   * @param steering_angle steering angle in radians
   *
   * @return std::pair<double, double> translational velocity, rotational velocity
   */
  virtual std::pair<double, double> wheels_velocities_to_cg(double rl_rpm,
                                                            [[maybe_unused]] double fl_rpm,
                                                            double rr_rpm,
                                                            [[maybe_unused]] double fr_rpm,
                                                            double steering_angle) = 0;

  /**
   * @brief Calculate rear axis coordinates
   *
   * @param cg center of gravity position
   * @param orientation orientation of the vehicle in radians relative to the world frame (ccw)
   * @param dist_cg_2_rear_axis distance between the center of gravity and the rear axis
   *
   * @return Point
   */
  virtual common_lib::structures::Position rear_axis_position(common_lib::structures::Position cg,
                                                              double orientation,
                                                              double dist_cg_2_rear_axis) = 0;

  /**
   * @brief Estimate wheel and motor velocities, as well as steering angle from the velocities of
   * the center of gravity
   *
   * @param cg_velocities vector of velocities on the Center of Gravity {velocity_x, velocity_y,
   * rotational_velocity} in m/s and rad/s respectively
   * @return Eigen::VectorXd vector of wheel speeds, steering angle and motor speed
   * {fl_rpm, fr_rpm, rl_rpm, rr_rpm, steering_angle, motor_rpm}
   */
  virtual Eigen::VectorXd cg_velocity_to_wheels(Eigen::Vector3d& cg_velocities) = 0;

  /**
   * @brief jacobian of the function cg_velocity_to_wheels with respect to the velocities of the
   * center of gravity
   *
   * @details Each entry at row i and column j of the resulting matrix is the partial derivative of
   * the i-th element of the output of function cg_velocity_to_wheels with respect to the j-th
   * element of the vector cg_velocities
   *
   * @param cg_velocities vector of velocities on the Center of Gravity {velocity_x, velocity_y,
   * rotational_velocity} in m/s and rad/s respectively
   * @return Eigen::MatrixXd jacobian matrix of the function cg_velocity_to_wheels (dimensions: 6x3)
   */
  virtual Eigen::MatrixXd jacobian_cg_velocity_to_wheels(Eigen::Vector3d& cg_velocities) = 0;
};