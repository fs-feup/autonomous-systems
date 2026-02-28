#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

/**
 * @brief Base class for velocity process models, used to predict the next velocities
 * based on previous velocities and accelerations.
 */
class BaseVelocityProcessModel {
public:
  BaseVelocityProcessModel() = default;

  /**
   * @brief Returns the next velocities based on the previous velocities and accelerations.
   * @param previous_velocities The velocities at the previous time step.
   * @param accelerations The accelerations applied during the time interval.
   * @param time_interval The time interval over which the accelerations are applied.
   * @return The predicted velocities at the next time step.
   */
  virtual Eigen::Vector3d get_next_velocities(const Eigen::Vector3d& previous_velocities,
                                              const Eigen::Vector3d& accelerations,
                                              const double time_interval) = 0;

  /**
   * @brief Returns the Jacobian of the velocity process model.
   * @param previous_velocities The velocities at the previous time step.
   * @param accelerations The accelerations applied during the time interval.
   * @param time_interval The time interval over which the accelerations are applied.
   * @return The Jacobian matrix of the velocity process model.
   */
  virtual Eigen::Matrix3d get_jacobian_velocities(
      const Eigen::Vector3d& previous_velocities,
      [[maybe_unused]] const Eigen::Vector3d& accelerations, const double time_interval) = 0;

  /**
   * @brief Returns the Jacobian of the velocity process model with respect to the accelerations.
   * @param previous_velocities The velocities at the previous time step.
   * @param accelerations The accelerations applied during the time interval.
   * @param time_interval The time interval over which the accelerations are applied.
   * @return The Jacobian matrix of the velocity process model with respect to the accelerations
   */
  virtual Eigen::Matrix3d get_jacobian_sensor_data(
      [[maybe_unused]] const Eigen::Vector3d& previous_velocities,
      [[maybe_unused]] const Eigen::Vector3d& accelerations, const double time_interval) = 0;
};