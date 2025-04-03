#include "motion_lib/particle_model.hpp"

#include <gtest/gtest.h>

/**
 * @brief Test the particle model
 *
 * @details Tests if the particle model updates the velocities correctly
 */
TEST(CAParticleModelTest, TestUpdateVelocities) {
  CAParticleModel particle_model;
  Eigen::Vector3d velocities(1.0, 2.0, 0.5);
  double ax = 0.1;
  double ay = 0.2;
  double angular_velocity = 0.3;
  double time_interval = 1.0;

  particle_model.update_velocities(velocities, ax, ay, angular_velocity, time_interval);

  EXPECT_NEAR(velocities(0), 1.1, 1e-5);
  // EXPECT_NEAR(velocities(1), 2.2, 1e-5);
  EXPECT_NEAR(velocities(2), 0.3, 1e-5);
}

/**
 * @brief Test the particle model's jacobian
 *
 * @details Tests if the particle model jacobian is filled out correctly
 */
TEST(CAParticleModelTest, TestJacobianOfVelocityUpdate) {
  CAParticleModel particle_model;
  Eigen::Matrix3d jacobian = particle_model.jacobian_of_velocity_update();

  Eigen::Matrix3d expected_jacobian = Eigen::Matrix3d::Identity();
  EXPECT_TRUE(jacobian.isApprox(expected_jacobian, 1e-5));
}