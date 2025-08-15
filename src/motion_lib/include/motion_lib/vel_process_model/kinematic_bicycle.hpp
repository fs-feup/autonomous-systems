#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/structures/velocities.hpp"
#include "motion_lib/vel_process_model/base_vel_process_model.hpp"

class KinematicBicycle : public BaseVelocityProcessModel {
public:
  KinematicBicycle(const common_lib::car_parameters::CarParameters& params)
      : BaseVelocityProcessModel(params) {}
  /**
   * @brief Predicts the next velocities based on the previous velocities and potentially other
   * measurements.
   * @param previous_velocities The velocities at the previous time step.
   * @param measurements ax, ay and steering angle.
   * @param time_interval The time interval over which the accelerations are applied.
   * @return The predicted velocities at the next time step.
   */
  Eigen::Vector3d get_next_velocities(const Eigen::Vector3d& previous_velocities,
                                      const Eigen::Vector3d& measurements,
                                      const double time_interval) override;
  /**
   * @brief Get the jacobian of the get_next_velocities function relative to the
   * previous_velocities.
   */
  Eigen::Matrix3d get_jacobian_velocities(const Eigen::Vector3d& previous_velocities,
                                          [[maybe_unused]] const Eigen::Vector3d& imu_data,
                                          const double time_interval) override;
};
