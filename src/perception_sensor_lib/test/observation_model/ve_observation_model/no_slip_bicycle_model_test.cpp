#include "perception_sensor_lib/observation_model/ve_observation_model/no_slip_bicycle_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

/* ----------------------- ODOMETRY MODEL -------------------------*/

/**
 * @brief Test a regular case of the conversion from wheel velocities to cg velocities
 *
 */
TEST(NoSlipBicycleModelTest, TestCgVelocityToWheels) {
  common_lib::car_parameters::CarParameters car_parameters_;
  car_parameters_.dist_cg_2_rear_axis = 1.0;
  car_parameters_.wheelbase = 2.5;
  car_parameters_.wheel_diameter = 0.6;
  car_parameters_.gear_ratio = 4.0;
  NoSlipBicycleModel bicycle_model = NoSlipBicycleModel(car_parameters_);
  Eigen::Vector3d cg_velocities(10.0, 2.0, 0.1);  // Example velocities
  Eigen::VectorXd observations = bicycle_model.expected_observations(cg_velocities);

  ASSERT_EQ(observations.size(), 6);

  EXPECT_NEAR(observations(0), 325.584, 0.01);   // front_wheels_rpm
  EXPECT_NEAR(observations(1), 325.584, 0.01);   // front_wheels_rpm
  EXPECT_NEAR(observations(2), 324.004, 0.01);   // rear_wheels_rpm
  EXPECT_NEAR(observations(3), 324.004, 0.01);   // rear_wheels_rpm
  EXPECT_NEAR(observations(4), 0.02450, 0.001);  // steering_angle
  EXPECT_NEAR(observations(5), 1296.02, 0.01);   // motor_rpm
}

/**
 * @brief Test the conversion from cg velocities to wheel velocities
 * when the velocity in x is negative (the car is moving backwards)
 *
 */
TEST(NoSlipBicycleModelTest, TestCgVelocityToWheelsNegativeVx) {
  common_lib::car_parameters::CarParameters car_parameters_;
  car_parameters_.dist_cg_2_rear_axis = 1.0;
  car_parameters_.wheelbase = 2.5;
  car_parameters_.wheel_diameter = 0.6;
  car_parameters_.gear_ratio = 4.0;
  NoSlipBicycleModel bicycle_model = NoSlipBicycleModel(car_parameters_);
  Eigen::Vector3d cg_velocities(-10.0, 2.0, 0.1);  // Example velocities
  Eigen::VectorXd observations = bicycle_model.expected_observations(cg_velocities);

  // Check the size of the observations vector
  ASSERT_EQ(observations.size(), 6);

  EXPECT_NEAR(observations(0), -325.584, 0.01);  // front_wheels_rpm
  EXPECT_NEAR(observations(1), -325.584, 0.01);  // front_wheels_rpm
  EXPECT_NEAR(observations(2), -324.004, 0.01);  // rear_wheels_rpm
  EXPECT_NEAR(observations(3), -324.004, 0.01);  // rear_wheels_rpm
  EXPECT_NEAR(observations(4), 0.02450, 0.001);  // steering_angle
  EXPECT_NEAR(observations(5), -1296.02, 0.01);  // motor_rpm
}

/**
 * @brief Test the conversion from cg velocities to wheel velocities
 * when the velocity in x is null and the other velocities are negative
 *
 */
TEST(NoSlipBicycleModelTest, TestCgVelocityToWheelsZeroVx) {
  common_lib::car_parameters::CarParameters car_parameters_;
  car_parameters_.dist_cg_2_rear_axis = 1.0;
  car_parameters_.wheelbase = 2.5;
  car_parameters_.wheel_diameter = 0.6;
  car_parameters_.gear_ratio = 4.0;
  NoSlipBicycleModel bicycle_model = NoSlipBicycleModel(car_parameters_);
  Eigen::Vector3d cg_velocities(0.0, -2.0, -0.1);  // Example velocities
  Eigen::VectorXd observations = bicycle_model.expected_observations(cg_velocities);

  // Check the size of the observations vector
  ASSERT_EQ(observations.size(), 6);

  EXPECT_NEAR(observations(0), 68.4366, 0.01);    // front_wheels_rpm
  EXPECT_NEAR(observations(1), 68.4366, 0.01);    // front_wheels_rpm
  EXPECT_NEAR(observations(2), 60.47888, 0.01);   // rear_wheels_rpm
  EXPECT_NEAR(observations(3), 60.47888, 0.01);   // rear_wheels_rpm
  EXPECT_NEAR(observations(4), -0.1243, 0.001);   // steering_angle
  EXPECT_NEAR(observations(5), 241.91552, 0.01);  // motor_rpm
}

/**
 * @brief Test the conversion from cg velocities to wheel velocities
 * when the velocities are null
 *
 */
TEST(NoSlipBicycleModelTest, TestCgVelocityToWheelsZeroVy) {
  common_lib::car_parameters::CarParameters car_parameters_;
  car_parameters_.dist_cg_2_rear_axis = 1.0;
  car_parameters_.wheelbase = 2.5;
  car_parameters_.wheel_diameter = 0.6;
  car_parameters_.gear_ratio = 4.0;
  NoSlipBicycleModel bicycle_model = NoSlipBicycleModel(car_parameters_);
  Eigen::Vector3d cg_velocities(0.0, 0.0, 0.0);  // Example velocities
  Eigen::VectorXd observations = bicycle_model.expected_observations(cg_velocities);

  // Check the size of the observations vector
  ASSERT_EQ(observations.size(), 6);

  EXPECT_NEAR(observations(0), 0, 0.01);   // front_wheels_rpm
  EXPECT_NEAR(observations(1), 0, 0.01);   // front_wheels_rpm
  EXPECT_NEAR(observations(2), 0, 0.01);   // rear_wheels_rpm
  EXPECT_NEAR(observations(3), 0, 0.01);   // rear_wheels_rpm
  EXPECT_NEAR(observations(4), 0, 0.001);  // steering_angle
  EXPECT_NEAR(observations(5), 0, 0.01);   // motor_rpm
}

/**
 * @brief Test the jacobian of the conversion from cg velocities to wheel velocities
 * with regular values of the velocities
 *
 */
TEST(NoSlipBicycleModelTest, TestJacobianCgVelocityToWheels) {
  // Initialize car parameters
  common_lib::car_parameters::CarParameters car_parameters;
  car_parameters.dist_cg_2_rear_axis = 1.0;
  car_parameters.wheelbase = 2.5;
  car_parameters.wheel_diameter = 0.6;
  car_parameters.gear_ratio = 4.0;

  // Initialize NoSlipBicycleModel with car parameters
  NoSlipBicycleModel bicycle_model(car_parameters);

  // Example velocities
  Eigen::Vector3d cg_velocities(10.0, 2.0, 0.1);

  // Calculate the Jacobian
  Eigen::MatrixXd jacobian = bicycle_model.expected_observations_jacobian(cg_velocities);

  // Check the size of the Jacobian matrix
  ASSERT_EQ(jacobian.rows(), 6);
  ASSERT_EQ(jacobian.cols(), 3);

  EXPECT_NEAR(jacobian(0, 0), 31.1199, 0.01);
  EXPECT_NEAR(jacobian(0, 1), 6.69077, 0.01);
  EXPECT_NEAR(jacobian(0, 2), 10.0362, 0.01);
  EXPECT_NEAR(jacobian(1, 0), 31.1199, 0.01);
  EXPECT_NEAR(jacobian(1, 1), 6.69077, 0.01);
  EXPECT_NEAR(jacobian(1, 2), 10.0362, 0.01);
  EXPECT_NEAR(jacobian(2, 0), 31.2715, 0.01);
  EXPECT_NEAR(jacobian(2, 1), 5.94159, 0.01);
  EXPECT_NEAR(jacobian(2, 2), -5.9416, 0.01);
  EXPECT_NEAR(jacobian(3, 0), 31.2715, 0.01);
  EXPECT_NEAR(jacobian(3, 1), 5.94159, 0.01);
  EXPECT_NEAR(jacobian(3, 2), -5.9416, 0.01);
  EXPECT_NEAR(jacobian(4, 0), -0.002355, 0.01);
  EXPECT_NEAR(jacobian(4, 1), -0.00047, 0.01);
  EXPECT_NEAR(jacobian(4, 2), 0.24499, 0.01);
  EXPECT_NEAR(jacobian(5, 0), 125.086, 0.01);
  EXPECT_NEAR(jacobian(5, 1), 23.7664, 0.01);
  EXPECT_NEAR(jacobian(5, 2), -23.7664, 0.01);
}

/**
 * @brief Test the jacobian of the conversion from cg velocities to wheel velocities
 * when the velocity in x is negative (the car is moving backwards), the velocity in y is null
 * and the angular velocity is negative
 *
 */
TEST(NoSlipBicycleModelTest, TestJacobianCgVelocityToWheelsZeroVy) {
  // Initialize car parameters
  common_lib::car_parameters::CarParameters car_parameters;
  car_parameters.dist_cg_2_rear_axis = 1.0;
  car_parameters.wheelbase = 2.5;
  car_parameters.wheel_diameter = 0.6;
  car_parameters.gear_ratio = 4.0;

  // Initialize NoSlipBicycleModel with car parameters
  NoSlipBicycleModel bicycle_model(car_parameters);

  // Example velocities
  Eigen::Vector3d cg_velocities(-10.0, 0, -0.1);

  // Calculate the Jacobian
  Eigen::MatrixXd jacobian = bicycle_model.expected_observations_jacobian(cg_velocities);

  // Check the size of the Jacobian matrix
  ASSERT_EQ(jacobian.rows(), 6);
  ASSERT_EQ(jacobian.cols(), 3);

  EXPECT_NEAR(jacobian(0, 0), 31.8274082, 0.01);
  EXPECT_NEAR(jacobian(0, 1), 0.47741112, 0.01);
  EXPECT_NEAR(jacobian(0, 2), 0.71611668, 0.01);
  EXPECT_NEAR(jacobian(1, 0), 31.8274082, 0.01);
  EXPECT_NEAR(jacobian(1, 1), 0.47741112, 0.01);
  EXPECT_NEAR(jacobian(1, 2), 0.71611668, 0.01);
  EXPECT_NEAR(jacobian(2, 0), 31.8293971, 0.01);
  EXPECT_NEAR(jacobian(2, 1), -0.3182939, 0.01);
  EXPECT_NEAR(jacobian(2, 2), 0.31829397, 0.01);
  EXPECT_NEAR(jacobian(3, 0), 31.8293971, 0.01);
  EXPECT_NEAR(jacobian(3, 1), -0.3182939, 0.01);
  EXPECT_NEAR(jacobian(3, 2), 0.31829397, 0.01);
  EXPECT_NEAR(jacobian(4, 0), -0.00149966, 0.01);
  EXPECT_NEAR(jacobian(4, 1), 0.0, 0.01);
  EXPECT_NEAR(jacobian(4, 2), 0.24984, 0.01);
  EXPECT_NEAR(jacobian(5, 0), 127.3175884, 0.01);
  EXPECT_NEAR(jacobian(5, 1), -1.27316, 0.01);
  EXPECT_NEAR(jacobian(5, 2), 1.27316, 0.01);
}