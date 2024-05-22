#include "point_solver/psolver.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "node_/node_control.hpp"
#include "test/include/utils.hpp"

/**
 * @brief Test Point Solver - update_closest_point()
 */
TEST(PointSolverTests, Test_update_closest_point_1) {
  auto pathpoint_array = create_path_msg("track1");

  Control ctrl;
  Point rear_axis = Point(47.0, -13.0);
  Point expected_point = Point(46.5, -12.37);
  int expected_id = 76;

  auto [path, rear_axis_point] = ctrl.point_solver_.update_closest_point(pathpoint_array, rear_axis);

  EXPECT_EQ(path.x_, expected_point.x_);
  EXPECT_EQ(path.y_, expected_point.y_);
  EXPECT_EQ(rear_axis_point, expected_id);
}

/**
 * @brief Test Point Solver - update_lookahead_point()
 */

TEST(PointSolverTests, Test_update_lookahead_point_1) {
  auto pathpoint_array = create_path_msg("track1");

  double ld = 3;
  double k = 3;
  double ld_margin = 0.1;

  Control ctrl;

  ctrl.k_ = k;
  ctrl.ld_margin_ = ld_margin;
  Point rear_axis = Point(47.0, -13.0);
  // Point cg = Point(47.80, -12.43);
  // Point closest_point = Point(46.5, -12.37);
  int closest_point_id = 76;
  Point expected_point = Point(48.75, -10.25);

  auto [result_point, result_velocity, result_error] = ctrl.point_solver_.update_lookahead_point(
      pathpoint_array, rear_axis, closest_point_id, ld, ld_margin);

  EXPECT_EQ(result_point.x_, expected_point.x_);
  EXPECT_EQ(result_point.y_, expected_point.y_);
  EXPECT_FALSE(result_error);
}