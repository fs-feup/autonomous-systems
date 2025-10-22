#pragma once

#include <vector>

#include "common_lib/structures/pose.hpp"

class TrajectoryCalculator {
public:
  /**
   * @brief Get the trajectory estimate object
   *
   * @return std::vector<common_lib::structures::Pose>
   */
  virtual std::vector<common_lib::structures::Pose> get_trajectory_estimate() = 0;
};