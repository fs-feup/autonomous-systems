#include "pure_pursuit/pp.hpp"

#include "gtest/gtest.h"

/**
 * @brief Test PurePursuit - calculate_alpha()
 */
TEST(PurePursuitTests, Test_calculate_alpha) {
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