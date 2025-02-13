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
   * @param cg
   * @param orientation
   * @param dist_cg_2_rear_axis
   *
   * @return Point
   */
  virtual common_lib::structures::Position rear_axis_position(common_lib::structures::Position cg,
                                                              double orientation,
                                                              double dist_cg_2_rear_axis) = 0;

  /**
   * @brief Estimate observations assuming a bicycle model and a set of parameters
   *
   * @param cg_velocities vector of velocities on the Center of Gravity {velocity_x, velocity_y,
   * rotational_velocity}
   * @param wheel_base distance between the front and rear wheels
   * @param weight_distribution_front percentage of the vehicle's weight on the front wheels
   * @param gear_ratio rotations of the motor for each rotation of the rear wheels
   * @param wheel_radius radius of the rear wheels
   * @return Eigen::VectorXd vector of observations {fl_rpm, fr_rpm, rl_rpm, rr_rpm, steering_angle,
   * motor_rpm}
   */
  virtual Eigen::VectorXd cg_velocity_to_wheels(Eigen::Vector3d& cg_velocities) = 0;

  /**
   * @brief jacobian of the function estimate_observations with respect to the state
   *
   * @param cg_velocities vector of velocities on the Center of Gravity {velocity_x, velocity_y,
   * rotational_velocity}
   * @param wheel_base distance between the front and rear wheels
   * @param weight_distribution_front percentage of the vehicle's weight on the front wheels
   * @param gear_ratio rotations of the motor for each rotation of the rear wheels
   * @param wheel_radius radius of the rear wheels
   * @return Eigen::MatrixXd jacobian matrix of the function estimate_observations (6x3)
   */
  virtual Eigen::MatrixXd jacobian_cg_velocity_to_wheels(Eigen::Vector3d& cg_velocities) = 0;
};