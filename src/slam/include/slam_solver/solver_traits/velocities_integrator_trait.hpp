#pragma once

#include "common_lib/structures/velocities.hpp"

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