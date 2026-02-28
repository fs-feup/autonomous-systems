#pragma once

#include "common_lib/structures/pose.hpp"

/**
 * @brief Trait class for odometry integration in SLAM solvers
 * @details This trait provides an interface for SLAM solvers to integrate odometry data,
 * allowing them to utilize odometry information for improved pose estimation.
 */
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