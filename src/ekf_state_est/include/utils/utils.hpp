#pragma once

#include <utility>
#include "common_lib/maths/transformations.hpp"
#include <memory>
#include "common_lib/car_parameters/car_parameters.hpp"

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
std::pair<double, double> wheels_velocities_to_cg(std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters, 
  double rl_rpm, [[maybe_unused]] double fl_rpm, double rr_rpm, [[maybe_unused]] double fr_rpm, double steering_angle);
