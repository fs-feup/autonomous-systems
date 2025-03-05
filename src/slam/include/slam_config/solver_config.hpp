#pragma once

#include "slam_config/general_config.hpp"

/**
 * @brief Parameters for the SLAM solver
 *
 */
struct SLAMSolverParameters {
  double observation_x_noise_ = 0.01;  //< Noise in the x direction of the observations' position
  double observation_y_noise_ = 0.01;  //< Noise in the y direction of the observations' position

  SLAMSolverParameters() = default;

  SLAMSolverParameters(const SLAMParameters& params);

  ~SLAMSolverParameters() = default;
};