#include "motion_lib/s2v_model/bicycle_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

/* ----------------------- ODOMETRY MODEL -------------------------*/

/**
 * @brief Tests the conversion from wheel revolutions
 * to velocities using the bycicle model
 *
 */
TEST(ODOMETRY_SUBSCRIBER, CONVERSION_TEST) {
  // Straight Line
  BicycleModel bicycle_model = BicycleModel(common_lib::car_parameters::CarParameters());
  double rl_speed = 60;
  double rr_speed = 60;
  double fl_speed = 60;
  double fr_speed = 60;
  double steering_angle = 0;
  std::pair<double, double> velocity_data =
      bicycle_model.wheels_velocities_to_cg(rl_speed, rr_speed, fl_speed, fr_speed, steering_angle);
  EXPECT_NEAR(velocity_data.first, 1.5708, 0.0001);
  EXPECT_DOUBLE_EQ(velocity_data.second, 0);

  // Curving left
  rl_speed = 60;
  rr_speed = 60;
  fl_speed = 60;
  fr_speed = 60;
  steering_angle = M_PI / 8;
  velocity_data =
      bicycle_model.wheels_velocities_to_cg(rl_speed, rr_speed, fl_speed, fr_speed, steering_angle);
  EXPECT_GE(velocity_data.first, 1.5708);
  EXPECT_LE(velocity_data.first, 1.5708 * 2);
  EXPECT_LE(velocity_data.second, M_PI);
  EXPECT_GE(velocity_data.second, M_PI / 8);

  // Curving right
  rl_speed = 60;
  rr_speed = 60;
  fl_speed = 60;
  fr_speed = 60;
  steering_angle = -M_PI / 8;
  velocity_data =
      bicycle_model.wheels_velocities_to_cg(rl_speed, rr_speed, fl_speed, fr_speed, steering_angle);
  EXPECT_GE(velocity_data.first, 1.5708);
  EXPECT_LE(velocity_data.first, 1.5708 * 2);
  EXPECT_GE(velocity_data.second, -M_PI);
  EXPECT_LE(velocity_data.second, -M_PI / 8);
}

TEST(BicycleModelTest, TestCgVelocityToWheels) {
  common_lib::car_parameters::CarParameters car_parameters_;
  car_parameters_.dist_cg_2_rear_axis = 1.0;
  car_parameters_.wheelbase = 2.5;
  car_parameters_.wheel_diameter = 0.6;
  car_parameters_.gear_ratio = 4.0;
  BicycleModel bicycle_model = BicycleModel(car_parameters_);
  Eigen::Vector3d cg_velocities(10.0, 2.0, 0.1);  // Example velocities
  Eigen::VectorXd observations = bicycle_model.cg_velocity_to_wheels(cg_velocities);

  ASSERT_EQ(observations.size(), 6);

  EXPECT_NEAR(observations(0), 325.584, 0.01);    // front_wheels_rpm
  EXPECT_NEAR(observations(1), 325.584, 0.01);    // front_wheels_rpm
  EXPECT_NEAR(observations(2), 324.004, 0.01);    // rear_wheels_rpm
  EXPECT_NEAR(observations(3), 324.004, 0.01);    // rear_wheels_rpm
  EXPECT_NEAR(observations(4), 0.211776, 0.001);  // steering_angle
  EXPECT_NEAR(observations(5), 1296.02, 0.01);    // motor_rpm
}

TEST(BicycleModelTest, TestCgVelocityToWheelsNegativeVx) {
  common_lib::car_parameters::CarParameters car_parameters_;
  car_parameters_.dist_cg_2_rear_axis = 1.0;
  car_parameters_.wheelbase = 2.5;
  car_parameters_.wheel_diameter = 0.6;
  car_parameters_.gear_ratio = 4.0;
  BicycleModel bicycle_model = BicycleModel(car_parameters_);
  Eigen::Vector3d cg_velocities(-10.0, 2.0, 0.1);  // Example velocities
  Eigen::VectorXd observations = bicycle_model.cg_velocity_to_wheels(cg_velocities);

  // Check the size of the observations vector
  ASSERT_EQ(observations.size(), 6);

  // Check the values of the observations (these values are just examples, you should replace them
  // with expected values)
  EXPECT_NEAR(observations(0), -325.584, 0.01);    // front_wheels_rpm
  EXPECT_NEAR(observations(1), -325.584, 0.01);    // front_wheels_rpm
  EXPECT_NEAR(observations(2), -324.004, 0.01);    // rear_wheels_rpm
  EXPECT_NEAR(observations(3), -324.004, 0.01);    // rear_wheels_rpm
  EXPECT_NEAR(observations(4), -0.211776, 0.001);  // steering_angle
  EXPECT_NEAR(observations(5), -1296.02, 0.01);    // motor_rpm
}

TEST(BicycleModelTest, TestCgVelocityToWheelsZeroVx) {
  common_lib::car_parameters::CarParameters car_parameters_;
  car_parameters_.dist_cg_2_rear_axis = 1.0;
  car_parameters_.wheelbase = 2.5;
  car_parameters_.wheel_diameter = 0.6;
  car_parameters_.gear_ratio = 4.0;
  BicycleModel bicycle_model = BicycleModel(car_parameters_);
  Eigen::Vector3d cg_velocities(0.0, 2.0, 0.1);  // Example velocities
  Eigen::VectorXd observations = bicycle_model.cg_velocity_to_wheels(cg_velocities);

  // Check the size of the observations vector
  ASSERT_EQ(observations.size(), 6);

  // Check the values of the observations (these values are just examples, you should replace them
  // with expected values)
  EXPECT_NEAR(observations(0), 68.4366, 0.01);  // front_wheels_rpm
  EXPECT_NEAR(observations(1), 68.4366, 0.01);  // front_wheels_rpm
  EXPECT_NEAR(observations(2), 60.4789, 0.01);  // rear_wheels_rpm
  EXPECT_NEAR(observations(3), 60.4789, 0.01);  // rear_wheels_rpm
  EXPECT_NEAR(observations(4), 0.0, 0.001);     // steering_angle
  EXPECT_NEAR(observations(5), 241.916, 0.01);  // motor_rpm
}

TEST(BicycleModelTest, TestJacobianCgVelocityToWheels) {
  // Initialize car parameters
  common_lib::car_parameters::CarParameters car_parameters;
  car_parameters.dist_cg_2_rear_axis = 1.0;
  car_parameters.wheelbase = 2.5;
  car_parameters.wheel_diameter = 0.6;
  car_parameters.gear_ratio = 4.0;

  // Initialize BicycleModel with car parameters
  BicycleModel bicycle_model(car_parameters);

  // Example velocities
  Eigen::Vector3d cg_velocities(10.0, 2.0, 0.1);

  // Calculate the Jacobian
  Eigen::MatrixXd jacobian = bicycle_model.jacobian_cg_velocity_to_wheels(cg_velocities);

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
  EXPECT_NEAR(jacobian(4, 0), -0.02055, 0.01);
  EXPECT_NEAR(jacobian(4, 1), 0.095582, 0.01);
  EXPECT_NEAR(jacobian(4, 2), 0.143373, 0.01);
  EXPECT_NEAR(jacobian(5, 0), 125.086, 0.01);
  EXPECT_NEAR(jacobian(5, 1), 23.7664, 0.01);
  EXPECT_NEAR(jacobian(5, 2), -23.7664, 0.01);
}
