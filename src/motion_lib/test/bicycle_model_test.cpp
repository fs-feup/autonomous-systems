#include "bicycle_model.hpp"

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

  double rl_speed = 60;
  double rr_speed = 60;
  double fl_speed = 60;
  double fr_speed = 60;
  double steering_angle = 0;
  std::pair<double, double> velocity_data =
      motion_lib::bicycle_model::odometry_to_velocities_transform(rl_speed, rr_speed, fl_speed,
                                                                  fr_speed, steering_angle);
  EXPECT_NEAR(velocity_data.first, 1.5708, 0.0001);
  EXPECT_DOUBLE_EQ(velocity_data.second, 0);

  // Curving left
  rl_speed = 60;
  rr_speed = 60;
  fl_speed = 60;
  fr_speed = 60;
  steering_angle = M_PI / 8;
  velocity_data = motion_lib::bicycle_model::odometry_to_velocities_transform(
      rl_speed, rr_speed, fl_speed, fr_speed, steering_angle);
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
  velocity_data = motion_lib::bicycle_model::odometry_to_velocities_transform(
      rl_speed, rr_speed, fl_speed, fr_speed, steering_angle);
  EXPECT_GE(velocity_data.first, 1.5708);
  EXPECT_LE(velocity_data.first, 1.5708 * 2);
  EXPECT_GE(velocity_data.second, -M_PI);
  EXPECT_LE(velocity_data.second, -M_PI / 8);
}