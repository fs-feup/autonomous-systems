#pragma once

#include <vector>

#include "common_lib/structures/pose.hpp"

/**
 * @brief Trait class for trajectory calculation in SLAM solvers
 * @details This trait provides an interface for SLAM solvers to calculate and retrieve
 * the trajectory estimate based on the integrated sensor data.
 */
class TrajectoryCalculator {
public:
  /**
   * @brief Get the trajectory estimate object
   *
   * @return std::vector<common_lib::structures::Pose>
   */
  virtual std::vector<common_lib::structures::Pose> get_trajectory_estimate() = 0;
};