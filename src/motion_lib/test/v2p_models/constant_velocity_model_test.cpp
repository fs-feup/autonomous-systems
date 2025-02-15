#include "motion_lib/v2p_models/constant_velocity_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - positive vx
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_1) {
  Eigen::Vector3f previous_pose(0, 0, 0);
  Eigen::Vector3f velocities(1, 0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 1, 0.000001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - positive vx
 * and positive vy
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_2) {
  Eigen::Vector3f previous_pose(0, 0, 0);
  Eigen::Vector3f velocities(1, 0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - negative vx
 * and positive vy
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_3) {
  Eigen::Vector3f previous_pose(0, 0, 0);
  Eigen::Vector3f velocities(-1, 0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - negative vx
 * and negative vy
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_4) {
  Eigen::Vector3f previous_pose(0, 0, 0);
  Eigen::Vector3f velocities(-1, -0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - positive vx
 * and negative vy
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_5) {
  Eigen::Vector3f previous_pose(0, 0, 0);
  Eigen::Vector3f velocities(1, -0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - positive vy
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_6) {
  Eigen::Vector3f previous_pose(0, 0, 0);
  Eigen::Vector3f velocities(0, 0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.2, 0.000001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - positive vy
 * and non origin pose
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_7) {
  Eigen::Vector3f previous_pose(1, 2, 0);
  Eigen::Vector3f velocities(0, 0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 1.2, 0.000001);
  EXPECT_NEAR(next_pose(1), 2, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a straight line movement in the x axis - positive vy
 * and non origin negative pose
 */
TEST(CONSTANT_VELOCITY_MODEL, X_AXIS_MOVEMENT_TEST_8) {
  Eigen::Vector3f previous_pose(-1, 2, 0);
  Eigen::Vector3f velocities(0, 0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), -0.8, 0.000001);
  EXPECT_NEAR(next_pose(1), 2, 0.000001);
  EXPECT_NEAR(next_pose(2), 0, 0.000001);
}

/**
 * @brief Test the constant velocity model with a backwards movement in the x axis - negative vx
 */
TEST(CONSTANT_VELOCITY_MODEL, BACKWARDS_MOVEMENT_TEST_1) {
  Eigen::Vector3f previous_pose(0, 0, -M_PI);
  Eigen::Vector3f velocities(-1, 0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), -1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
}

/**
 * @brief Test the constant velocity model with a backwards movement in the x axis - positive vx
 * (paradox)
 */
TEST(CONSTANT_VELOCITY_MODEL, BACKWARDS_MOVEMENT_TEST_2) {
  Eigen::Vector3f previous_pose(0, 0, -M_PI);
  Eigen::Vector3f velocities(1, 0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), -1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
}

/**
 * @brief Test the constant velocity model with a backwards movement in the x axis - negative vy
 * (paradox)
 */
TEST(CONSTANT_VELOCITY_MODEL, BACKWARDS_MOVEMENT_TEST_3) {
  Eigen::Vector3f previous_pose(0, 0, -M_PI);
  Eigen::Vector3f velocities(1, -0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), -1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
}

/**
 * @brief Test the constant velocity model with a backwards movement in the x axis - negative
 * velocities
 */
TEST(CONSTANT_VELOCITY_MODEL, BACKWARDS_MOVEMENT_TEST_4) {
  Eigen::Vector3f previous_pose(0, 0, -M_PI);
  Eigen::Vector3f velocities(-1, -0.2, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), -1.02, 0.001);
  EXPECT_NEAR(next_pose(1), 0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - 2PI
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_1) {
  Eigen::Vector3f previous_pose(0, 0, 2 * M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), 0.0, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - 3PI
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_2) {
  Eigen::Vector3f previous_pose(0, 0, 3 * M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - -PI
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_3) {
  Eigen::Vector3f previous_pose(0, 0, -M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - -3PI
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_4) {
  Eigen::Vector3f previous_pose(0, 0, -3 * M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(abs(next_pose(2)), M_PI, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - 3PI/2
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_5) {
  Eigen::Vector3f previous_pose(0, 0, 1.5 * M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), -M_PI / 2, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - 5PI/2
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_6) {
  Eigen::Vector3f previous_pose(0, 0, 2.5 * M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), M_PI / 2, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - -5PI/2
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_7) {
  Eigen::Vector3f previous_pose(0, 0, -2.5 * M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), -M_PI / 2, 0.000001);
}

/**
 * @brief Test the constant velocity model for angle cap on the orientation update - -7PI/2
 */
TEST(CONSTANT_VELOCITY_MODEL, ORIENTATION_ANGLE_CAP_TEST_8) {
  Eigen::Vector3f previous_pose(0, 0, -3.5 * M_PI);
  Eigen::Vector3f velocities(0.0, 0.0, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0.0, 0.001);
  EXPECT_NEAR(next_pose(1), 0.0, 0.000001);
  EXPECT_NEAR(next_pose(2), M_PI / 2, 0.000001);
}

/**
 * @brief Test the constant velocity model in a straight line movement with a non-zero orientation
 */
TEST(CONSTANT_VELOCITY_MODEL, STRAIGHT_LINE_MOVEMENT_TEST_1) {
  Eigen::Vector3f previous_pose(1, 2, M_PI / 4);
  Eigen::Vector3f velocities(3, -0.7, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 3.18, 0.01);
  EXPECT_NEAR(next_pose(1), 4.18, 0.01);
  EXPECT_NEAR(next_pose(2), M_PI / 4, 0.000001);
}

/**
 * @brief Test the constant velocity model in a straight line movement with a negative orientation
 */
TEST(CONSTANT_VELOCITY_MODEL, STRAIGHT_LINE_MOVEMENT_TEST_2) {
  Eigen::Vector3f previous_pose(1, 2, -M_PI / 4);
  Eigen::Vector3f velocities(3, -0.7, 0);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 3.18, 0.01);
  EXPECT_NEAR(next_pose(1), -0.18, 0.01);
  EXPECT_NEAR(next_pose(2), -M_PI / 4, 0.000001);
}

/**
 * @brief Test the constant velocity model in a straight line movement with a perpendicular
 * orientation
 */
TEST(CONSTANT_VELOCITY_MODEL, STRAIGHT_LINE_MOVEMENT_TEST_3) {
  Eigen::Vector3f previous_pose(1, 2, -M_PI / 2);
  Eigen::Vector3f velocities(3, -0.7, 0);
  double delta_t = 2;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 1, 0.000001);
  EXPECT_NEAR(next_pose(1), -4.16, 0.01);
  EXPECT_NEAR(next_pose(2), -M_PI / 2, 0.000001);
}

/**
 * @brief Test the constant velocity model in a curviliniar movement
 */
TEST(CONSTANT_VELOCITY_MODEL, CURVILINEAR_MOVEMENT_TEST_1) {
  Eigen::Vector3f previous_pose(1, 2, 0);
  Eigen::Vector3f velocities(3, -0.7, -M_PI / 8);
  double delta_t = 1;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 3.85, 0.01);
  EXPECT_NEAR(next_pose(1), 0.82, 0.01);
  EXPECT_NEAR(next_pose(2), -M_PI / 8, 0.000001);
}

/**
 * @brief Test the constant velocity model in a curviliniar movement with a non-zero orientation
 */
TEST(CONSTANT_VELOCITY_MODEL, CURVILINEAR_MOVEMENT_TEST_2) {
  Eigen::Vector3f previous_pose(0, 0, M_PI / 4);
  Eigen::Vector3f velocities(3, 0.5, M_PI / 8);
  double delta_t = 2;
  ConstantVelocityModel model;
  Eigen::Vector3f next_pose = model.get_next_pose(previous_pose, velocities, delta_t);
  EXPECT_NEAR(next_pose(0), 0, 0.01);
  EXPECT_NEAR(next_pose(1), 6.08, 0.01);
  EXPECT_NEAR(next_pose(2), M_PI / 2, 0.000001);
}