#pragma once

#include "common_lib/structures/velocities.hpp"

/**
 * @brief Trait class for velocities integration in SLAM solvers
 * @details This trait provides an interface for SLAM solvers to integrate
 * velocity data, allowing them to utilize motion priors for improved pose estimation.
 */
class VelocitiesIntegratorTrait {
public:
  /**
   * @brief Add motion prior to the solver (prediction step)
   *
   * @param velocities Velocities of the robot
   */
  virtual void add_velocities(const common_lib::structures::Velocities& velocities) = 0;

  virtual ~VelocitiesIntegratorTrait() = default;
};