#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/structures/velocities.hpp"
#include "motion_lib/vel_process_model/base_vel_process_model.hpp"

class CAParticleModel : public BaseVelocityProcessModel {
public:
  CAParticleModel() = default;
  // /**
  //  * @brief Update velocities assuming period of time with constant accelerations acting in the
  //  * Center of Mass
  //  *
  //  * @param velocities size 3 vector with {velocity along x axis; velocity along y axis; angular
  //  * velocity around z axis}
  //  * @param ax acceleration along x axis
  //  * @param ay acceleration along y axis
  //  * @param angular_velocity angular velocity around z axis
  //  * @param time_interval time interval over which the update is applied
  //  */
  // void update_velocities(Eigen::Vector3d& velocities, double ax, double ay, double
  // angular_velocity,
  //                        double time_interval);
  // /**
  //  * @brief Compute the Jacobian matrix of the velocity update function
  //  */
  // Eigen::Matrix3d jacobian_of_velocity_update();

  Eigen::Vector3d get_next_velocities(const Eigen::Vector3d& previous_velocities,
                                      const Eigen::Vector3d& accelerations,
                                      const double time_interval) override;

  Eigen::Matrix3d get_jacobian_velocities(const Eigen::Vector3d& previous_velocities,
                                          [[maybe_unused]] const Eigen::Vector3d& accelerations,
                                          const double time_interval) override;
};
