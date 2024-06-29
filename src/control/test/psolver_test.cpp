#include "point_solver/psolver.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "node_/node_control.hpp"
#include "test/include/utils.hpp"

using namespace common_lib::structures;

/**
 * @brief Test Point Solver - update_closest_point()
 */
TEST(PointSolverTests, Test_update_closest_point_1) {
  auto pathpoint_array = create_path_msg("track1");

  PointSolver point_solver_(0);
  point_solver_.vehicle_pose_.rear_axis_ = Position(47.0, -13.0);
  Position expected_point = Position(46.5, -12.37);
  int expected_id = 76;

  auto [path, rear_axis_point, closest_point_velocity] = point_solver_.update_closest_point(pathpoint_array);

  EXPECT_EQ(path.x, expected_point.x);
  EXPECT_EQ(path.y, expected_point.y);
  EXPECT_EQ(rear_axis_point, expected_id);
}

/**
 * @brief Test Point Solver - update_lookahead_point()
 */


TEST(PointSolverTests, Test_update_lookahead_point_1) {
  auto pathpoint_array = create_path_msg("track1");

  PointSolver point_solver_(1.5);
  point_solver_.vehicle_pose_.rear_axis_ = Position(47.0, -13.0);
  int closest_point_id = 76;
  Position expected_point = Position(48.58, -10.44);

  auto [result_point, result_velocity, result_error] =
      point_solver_.update_lookahead_point(pathpoint_array, closest_point_id);

  EXPECT_NEAR(result_point.x, expected_point.x, 0.01);
  EXPECT_NEAR(result_point.y, expected_point.y, 0.01);
  EXPECT_FALSE(result_error);
}