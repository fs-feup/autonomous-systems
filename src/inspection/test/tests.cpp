#include <chrono>
#include <fstream>

#include "gtest/gtest.h"
#include "inspection_node/inspection_ros.hpp"
#include "rclcpp/rclcpp.hpp"

using testing::Eq;

/**
 * @brief round to n decimal places
 *
 * @param number number to round
 * @param n number of decimal places
 * @return double with n decimal places
 */
double round(double number, int n) {
  number *= pow(10, n);
  number = round(number);
  return number / pow(10, n);
}

/**
 * @brief test steering output is sinusoidal
 *
 */
TEST(STEERING, steering1) {
  auto new_inspection = std::make_unique<InspectionFunctions>(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  for (unsigned int i = 0; i < 260; i++) {
    double steering = new_inspection->calculate_steering(i);
    EXPECT_FLOAT_EQ(steering,
                    sin(i * 2 * M_PI / new_inspection->turning_period) * new_inspection->max_angle);
  }
}

/**
 * @brief test torque output control. Initial velocity greater than target.
 *
 */
TEST(TORQUE, torque1) {
  auto new_inspection = std::make_unique<InspectionFunctions>(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double initial_velocity = 5;
  // i represents time
  for (unsigned int i = 0; i < 260; i++) {
    initial_velocity += 0.1 * (new_inspection->calculate_throttle(initial_velocity));
  }
  EXPECT_NEAR(initial_velocity, 1, 0.01);
}

/**
 * @brief test torque output control. Initial velocity smaller than target.
 *
 */
TEST(TORQUE, torque2) {
  auto new_inspection = std::make_unique<InspectionFunctions>(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double initial_velocity = -4;
  // i represents time
  for (unsigned int i = 0; i < 260; i++) {
    initial_velocity += 0.1 * (new_inspection->calculate_throttle(initial_velocity));
  }
  EXPECT_NEAR(initial_velocity, 1, 0.01);
}

/**
 * @brief test torque output control. Initial velocity similar to ideal velocity.
 *
 */
TEST(TORQUE, torque3) {
  auto new_inspection = std::make_unique<InspectionFunctions>(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double initial_velocity = 1.1;
  // i represents time
  for (unsigned int i = 0; i < 260; i++) {
    initial_velocity += 0.1 * (new_inspection->calculate_throttle(initial_velocity));
  }
  EXPECT_NEAR(initial_velocity, 1, 0.01);
}

/**
 * @brief test control while oscilating the goal velocity.
 *
 */
TEST(TORQUE, torque4) {
  double max_angle = 3.14159265358979323846264338327950288 / 6.0;
  double ideal_velocity = 10.0;
  double turn_time = 0;
  double wheel_radius = 0.254;
  double gain = 0.5;
  double stop_time = 30;
  auto new_inspection = std::make_unique<InspectionFunctions>(
      max_angle, turn_time, wheel_radius, stop_time, true, gain, ideal_velocity);

  double current_velocity = 0;
  double max_velocity = -1;
  double min_velocity = 1000;
  int count = 0;
  // i represents time
  for (unsigned int i = 0; i < static_cast<unsigned int>(stop_time) * 10; i++) {
    // save the maximum and minimum velocities
    max_velocity = current_velocity > max_velocity ? current_velocity : max_velocity;
    min_velocity = current_velocity < min_velocity ? current_velocity : min_velocity;

    // update velocity
    current_velocity += 0.1 * (new_inspection->calculate_throttle(current_velocity));

    // check goal has been reached and flip the goal
    if (fabs(current_velocity - new_inspection->current_goal_velocity) < 0.2) {
      new_inspection->redefine_goal_velocity(current_velocity);
      count += 1;  // count how many times the ideal velocity changes
    }
  }
  EXPECT_GT(count, 2);  // test if the ideal velocity changes the right number of times
  EXPECT_EQ(round(min_velocity, 1), 0.0);
  EXPECT_DOUBLE_EQ(round(max_velocity, 1), 9.9);
}

/**
 * @brief test conversion from rpm to velocity with constants defined directly
 *
 */
TEST(CONVERSION, conversion1) {
  auto new_inspection = std::make_unique<InspectionFunctions>(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double rpm = 15;
  EXPECT_DOUBLE_EQ(0.39898226700590372, new_inspection->rpm_to_velocity(rpm));
}
