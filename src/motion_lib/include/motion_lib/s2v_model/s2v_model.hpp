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
  virtual common_lib::structures::Position rear_axis_position(
      const common_lib::structures::Position& cg, double orientation,
      double dist_cg_2_rear_axis) = 0;
};