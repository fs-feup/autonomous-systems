#include "motion_lib/vel_process_model/particle_model.hpp"

#include <gtest/gtest.h>

/**
 * @brief Test the particle model
 *
 * @details Tests if the particle model updates the velocities correctly
 */
TEST(CAParticleModelTest, TestUpdateVelocities) {
  CAParticleModel particle_model;
  Eigen::Vector3d velocities(1.0, 2.0, 0.5);
  Eigen::Vector3d accelerations(0.1, 0.2, 0.3);

  double time_interval = 1.0;

  Eigen::Vector3d result =
      particle_model.get_next_velocities(velocities, accelerations, time_interval);

  EXPECT_NEAR(result(0), 2.1, 1e-5);
  EXPECT_NEAR(result(1), 1.7, 1e-5);
  EXPECT_NEAR(result(2), 0.5, 1e-5);
}

/**
 * @brief Test the particle model's jacobian
 *
 * @details Tests if the particle model jacobian is filled out correctly
 */
TEST(CAParticleModelTest, TestJacobianOfVelocityUpdate) {
  CAParticleModel particle_model;
  Eigen::Vector3d velocities(1.0, 2.0, 0.5);
  Eigen::Vector3d accelerations(0.1, 0.2, 0.3);

  double time_interval = 1.0;
  Eigen::Matrix3d jacobian =
      particle_model.get_jacobian_velocities(velocities, accelerations, time_interval);

  EXPECT_FLOAT_EQ(jacobian(0, 0), 1.0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1.0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1.0);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0.5);
  EXPECT_FLOAT_EQ(jacobian(0, 2), 2.0);
  EXPECT_FLOAT_EQ(jacobian(1, 0), -0.5);
  EXPECT_FLOAT_EQ(jacobian(1, 2), -1.0);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0.0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0.0);
}