#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

class BaseVelocityProcessModel {
public:
  BaseVelocityProcessModel() = default;

  virtual Eigen::Vector3d get_next_velocities(const Eigen::Vector3d& previous_velocities,
                                              const Eigen::Vector3d& accelerations,
                                              const double time_interval) = 0;

  virtual Eigen::Matrix3d get_jacobian_velocities(
      const Eigen::Vector3d& previous_velocities,
      [[maybe_unused]] const Eigen::Vector3d& accelerations, const double time_interval) = 0;
};