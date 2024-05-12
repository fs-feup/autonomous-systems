#include "pure_pursuit/pp.hpp"

#include "gtest/gtest.h"

/**
 * @brief Test PurePursuit - calculate_alpha()
 * typical angle - path point left
 */
TEST(PurePursuitTests, Test_calculate_alpha_1) {
  double k = 0.5;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  Point vehicle_cg = Point(5, 4.46);
  Point vehicle_rear_wheel = Point(6, 2);
  Point lookahead_point = Point(1, 4);
  double rear_wheel_2_c_g = 2.655484889;

  double alpha =
      pp.calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point, rear_wheel_2_c_g);

  EXPECT_NEAR(0.804, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * typical angle - path point right
 */
TEST(PurePursuitTests, Test_calculate_alpha_2) {
  double k = 0.5;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  Point vehicle_cg = Point(3, 3);
  Point vehicle_rear_wheel = Point(3, 5);
  Point lookahead_point = Point(5, 1);
  double rear_wheel_2_c_g = 2;

  double alpha =
      pp.calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point, rear_wheel_2_c_g);

  EXPECT_NEAR(0.463, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * 90 degrees angle
 */
TEST(PurePursuitTests, Test_calculate_alpha_3) {
  double k = 0.5;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  Point vehicle_cg = Point(3, 3);
  Point vehicle_rear_wheel = Point(3, 5);
  Point lookahead_point = Point(5, 5);
  double dist_cg_2_rear_axis = 2;

  double alpha =
      pp.calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point, dist_cg_2_rear_axis);

  EXPECT_NEAR(1.570, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * 0 degrees angle
 */
TEST(PurePursuitTests, Test_calculate_alpha_4) {
  double k = 0.5;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  Point vehicle_cg = Point(3, 3);
  Point vehicle_rear_wheel = Point(3, 5);
  Point lookahead_point = Point(3, 0);
  double rear_wheel_2_c_g = 2;

  double alpha =
      pp.calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point, rear_wheel_2_c_g);

  EXPECT_NEAR(0, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * 180 degrees angle
 */
TEST(PurePursuitTests, Test_calculate_alpha_5) {
  double k = 0.5;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  Point vehicle_cg = Point(3, 3);
  Point vehicle_rear_wheel = Point(3, 5);
  Point lookahead_point = Point(3, 7);
  double rear_wheel_2_c_g = 2;

  double alpha =
      pp.calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point, rear_wheel_2_c_g);

  EXPECT_NEAR(3.141, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - pp_steering_control_law()
 */
TEST(PurePursuitTests, Test_pp_steering_control_law_1) {
  double k = 0.5;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  pp.vehicle_pose_.cg_ = Point(5, 4.46);
  pp.vehicle_pose_.rear_axis_ = Point(6, 2);
  pp.lookahead_point_ = Point(1, 4);
  pp.dist_cg_2_rear_axis_ = 2.655484889;

  double steering_cmd = pp.pp_steering_control_law();
  //  Alpha: 0.804189
  //  ld_: 5.38516
  EXPECT_NEAR(0.381, steering_cmd, 0.001);
}