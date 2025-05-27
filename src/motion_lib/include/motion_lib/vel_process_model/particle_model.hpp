#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/structures/velocities.hpp"
#include "motion_lib/vel_process_model/base_vel_process_model.hpp"

class CAParticleModel : public BaseVelocityProcessModel {
public:
  CAParticleModel() = default;

  /**
   * @brief Predicts the next velocities based on the previous velocities and accelerations from
   * IMU.
   * @param previous_velocities The velocities at the previous time step.
   * @param imu_data ax, ay and angular velocity (angular velocity not used).
   * @param time_interval The time interval over which the accelerations are applied.
   * @return The predicted velocities at the next time step.
   */
  Eigen::Vector3d get_next_velocities(const Eigen::Vector3d& previous_velocities,
                                      const Eigen::Vector3d& imu_data,
                                      const double time_interval) override;

  Eigen::Matrix3d get_jacobian_velocities(const Eigen::Vector3d& previous_velocities,
                                          [[maybe_unused]] const Eigen::Vector3d& imu_data,
                                          const double time_interval) override;
};
