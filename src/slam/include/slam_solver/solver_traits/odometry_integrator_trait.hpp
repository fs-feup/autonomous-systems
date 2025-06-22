#pragma once

#include "common_lib/structures/pose.hpp"

class OdometryIntegratorTrait {
public:
  /**
   * @brief Integrate odometry data into the SLAM solver
   *
   * @param pose_difference Pose difference in the form of [dx, dy, dtheta]
   */
  virtual void add_odometry(const common_lib::structures::Pose& pose_difference) = 0;

  virtual ~OdometryIntegratorTrait() = default;
};