#include "motion_lib/v2p_models/constant_velocity_turnrate_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

/**
 * @brief Test the constant velocity and turnrate model with a straight line movement in the x axis
 * - positive vx
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, STRAIGHT_LINE_MOVEMENT_TEST_1) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, 0);
  Eigen::Vector3d velocities(1, 0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 1, 0.000001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(0, 2), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_FLOAT_EQ(jacobian(1, 2), 1);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model with a straight line movement in the x axis
 * - positive vx
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, STRAIGHT_LINE_MOVEMENT_TEST_2) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, M_PI / 4);
  Eigen::Vector3d velocities(2, 0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 1.41, 0.01);
  EXPECT_NEAR(next_pose(1), 1.41, 0.01);
  EXPECT_NEAR(next_pose(2), M_PI / 4, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), -1.41, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 1.41, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model with a backwards movement in the x axis -
 * negative vx
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, BACKWARDS_MOVEMENT_TEST_1) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, 0.0);
  Eigen::Vector3d velocities(-1, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), -1.0, 0.000001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), 0.0, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), -1, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model with a backwards movement in the x axis -
 * positive vx (paradox)
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, BACKWARDS_MOVEMENT_TEST_2) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, M_PI);
  Eigen::Vector3d velocities(1, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), -1.00, 0.000001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), -1, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model with a backwards movement in the x axis -
 * negative vy (paradox)
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, BACKWARDS_MOVEMENT_TEST_3) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, -M_PI);
  Eigen::Vector3d velocities(-1, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 1.00, 0.000001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 1, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model for angle cap on the orientation update -
 * 2PI
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, ORIENTATION_ANGLE_CAP_TEST_1) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, 2 * M_PI);
  Eigen::Vector3d velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0.0, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model for angle cap on the orientation update -
 * 3PI
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, ORIENTATION_ANGLE_CAP_TEST_2) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, 3 * M_PI);
  Eigen::Vector3d velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model for angle cap on the orientation update -
 * -3PI
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, ORIENTATION_ANGLE_CAP_TEST_3) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, -3 * M_PI);
  Eigen::Vector3d velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model for angle cap on the orientation update -
 * 3PI/2
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, ORIENTATION_ANGLE_CAP_TEST_4) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, 1.5 * M_PI);
  Eigen::Vector3d velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), -M_PI / 2, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model for angle cap on the orientation update -
 * -5PI/2
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, ORIENTATION_ANGLE_CAP_TEST_5) {
  // Arrange
  Eigen::Vector3d previous_pose(0, 0, -2.5 * M_PI);
  Eigen::Vector3d velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), -M_PI / 2, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model in a curvilinear movement with a
 * non-zero orientation
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, CURVILINEAR_MOVEMENT_TEST_1) {
  // Arrange
  Eigen::Vector3d previous_pose(1, 2, M_PI / 4);
  Eigen::Vector3d velocities(3, 0, M_PI / 16);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 2.9, 0.01);
  EXPECT_NEAR(next_pose(1), 4.316, 0.01);
  EXPECT_NEAR(next_pose(2), 0.98, 0.01);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), -2.315, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 1.9, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model in a curvilinear movement with a negative
 * orientation
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, CURVILINEAR_MOVEMENT_TEST_2) {
  // Arrange
  Eigen::Vector3d previous_pose(1, 2, -M_PI / 4);
  Eigen::Vector3d velocities(3, 0, -M_PI / 8);
  double delta_t = 1;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 2.656, 0.01);
  EXPECT_NEAR(next_pose(1), -0.48, 0.01);
  EXPECT_NEAR(next_pose(2), -3 * M_PI / 8, 0.000001);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 2.48, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 1.656, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}

/**
 * @brief Test the constant velocity and turnrate model in a straight line movement with a
 * perpendicular orientation
 */
TEST(CONSTANT_VELOCITY_TURNRATE_MODEL, CURVILINEAR_MOVEMENT_TEST_3) {
  // Arrange
  Eigen::Vector3d previous_pose(1, 2, -M_PI / 2);
  Eigen::Vector3d velocities(3, 0, M_PI / 4);
  double delta_t = 2;
  ConstantVelocityTurnrateModel model;

  // Act
  Eigen::Vector3d next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  Eigen::Matrix3d jacobian = model.get_jacobian(previous_pose, velocities, delta_t);

  // Assert
  EXPECT_NEAR(next_pose(0), 4.820, 0.01);
  EXPECT_NEAR(next_pose(1), -1.820, 0.01);
  EXPECT_NEAR(next_pose(2), 0.0, 0.01);
  EXPECT_FLOAT_EQ(jacobian(0, 0), 1);
  EXPECT_FLOAT_EQ(jacobian(0, 1), 0);
  EXPECT_NEAR(jacobian(0, 2), 3.820, 0.01);
  EXPECT_FLOAT_EQ(jacobian(1, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(1, 1), 1);
  EXPECT_NEAR(jacobian(1, 2), 3.820, 0.01);
  EXPECT_FLOAT_EQ(jacobian(2, 0), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 1), 0);
  EXPECT_FLOAT_EQ(jacobian(2, 2), 1);
}