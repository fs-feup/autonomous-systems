#pragma once

#include <cmath>
#include <iostream>
#include <utility>

#include "common_lib/maths/transformations.hpp"
#include "motion_lib/s2v_model/s2v_model.hpp"

class NoRearWSSBicycleModel : public S2VModel {
  common_lib::car_parameters::CarParameters car_parameters_;

public:
  NoRearWSSBicycleModel(common_lib::car_parameters::CarParameters car_parameters)
      : car_parameters_(car_parameters) {}
  /**
   * @brief assumes a bicycle model and a set of parameters about the vehicle to calculate the
   * velocities of the center of mass given the velocities of the wheels
   *
   * @param rl_rpm rear left wheel velocity in rpm
   * @param fl_rpm front left wheel velocity in rpm
   * @param steering_angle steering angle in radians
   * @return std::pair<double, double> translational and rotational velocities of the center of mass
   * in m/s and rad/s respectively
   */
  std::pair<double, double> wheels_velocities_to_cg(double rl_rpm, [[maybe_unused]] double fl_rpm,
                                                    double rr_rpm, [[maybe_unused]] double fr_rpm,
                                                    double steering_angle) override;

  /**
   * @brief Calculates the position of the rear axis given the position of the center of mass, the
   * the orientation and the distance between the center of mass and the rear axis
   *
   * @param cg position of the center of mass in meters
   * @param orientation orientation of the vehicle relative to the world frame in radians
   * @param dist_cg_2_rear_axis disntance between the center of mass and the rear axis in meters
   * @return common_lib::structures::Position
   */
  common_lib::structures::Position rear_axis_position(const common_lib::structures::Position& cg,
                                                      double orientation,
                                                      double dist_cg_2_rear_axis) override;
};