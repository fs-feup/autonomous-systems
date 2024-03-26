#include <chrono>
#include <fstream>

#include "gtest/gtest.h"
#include "include/inspection_ros.hpp"
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
  InspectionFunctions *new_inspection = new InspectionFunctions(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  for (float i = 0; i < 26; i = i + 0.1) {
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
  InspectionFunctions *new_inspection = new InspectionFunctions(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double initial_velocity = 5;
  // i represents time
  for (float i = 0; i < 26; i = i + 0.1) {
    initial_velocity += 0.1 * (new_inspection->calculate_throttle(initial_velocity));
  }
  EXPECT_NEAR(initial_velocity, 1, 0.01);
}

/**
 * @brief test torque output control. Initial velocity smaller than target.
 *
 */
TEST(TORQUE, torque2) {
  InspectionFunctions *new_inspection = new InspectionFunctions(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double initial_velocity = -4;
  // i represents time
  for (float i = 0; i < 26; i = i + 0.1) {
    initial_velocity += 0.1 * (new_inspection->calculate_throttle(initial_velocity));
  }
  EXPECT_NEAR(initial_velocity, 1, 0.01);
}

/**
 * @brief test torque output control. Initial velocity similar to ideal velocity.
 *
 */
TEST(TORQUE, torque3) {
  InspectionFunctions *new_inspection = new InspectionFunctions(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double initial_velocity = 1.1;
  // i represents time
  for (float i = 0; i < 26; i = i + 0.1) {
    initial_velocity += 0.1 * (new_inspection->calculate_throttle(initial_velocity));
  }
  EXPECT_NEAR(initial_velocity, 1, 0.01);
}

/**
 * @brief test control while oscilating the goal velocity.
 *
 */
TEST(TORQUE, torque4) {
  double max_angle = 3.14159265358979323846264338327950288 / 6.0, ideal_velocity = 10.0,
         turn_time = 0, wheel_radius = 0.254, gain = 0.5, stop_time = 30;
  InspectionFunctions *new_inspection = new InspectionFunctions(
      max_angle, turn_time, wheel_radius, stop_time, false, gain, ideal_velocity);

  double current_velocity = 0, max_velocity = -1, min_velocity = 1000;
  bool accelerating = true;
  int count = 0;
  // i represents time
  for (float i = 0; i < stop_time; i = i + 0.1) {
    // save the maximum and minimum velocities
    max_velocity = current_velocity > max_velocity ? current_velocity : max_velocity;
    min_velocity = current_velocity < min_velocity ? current_velocity : min_velocity;

    // update velocity
    current_velocity += 0.1 * (new_inspection->calculate_throttle(current_velocity));

    // check goal has been reached and flip the goal
    if (round(current_velocity, 2) == new_inspection->ideal_velocity) {
      new_inspection->redefine_goal_velocity(accelerating ? 0.0 : 10.0);
      accelerating = false;
      count += 1;  // count how many times the ideal velocity changes
    }
  }
  delete new_inspection;
  EXPECT_GT(count, 2);  // test if the ideal velocity changes the right number of times
  EXPECT_EQ(round(min_velocity, 2), 0.0);
  EXPECT_DOUBLE_EQ(round(max_velocity, 2), 10.0);
}

/**
 * @brief test conversion from rpm to velocity with constants defined directly
 *
 */
TEST(CONVERSION, conversion1) {
  InspectionFunctions *new_inspection = new InspectionFunctions(
      3.14159265358979323846264338327950288 / 6.0, 4.0, 0.254, 26, false, 0.5, 1.0);
  double rpm = 15;
  EXPECT_DOUBLE_EQ(0.39898226700590372, new_inspection->rpm_to_velocity(rpm));
}
