#include "point_solver/psolver.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "test/include/utils.hpp"
#include "node_/node_control.hpp"

/**
 * @brief Test Point Solver - update_closest_point()
 */
TEST(PointSolverTests, Test_update_closest_point_1) {
  custom_interfaces::msg::PathPointArray path_msg = create_path_msg("track1");

  // Convert path_msg to ConstSharedPtr
  const custom_interfaces::msg::PathPointArray::ConstSharedPtr path_msg_ptr =
      std::make_shared<const custom_interfaces::msg::PathPointArray>(path_msg);


  Control ctrl;
  Point rear_axis = Point(47.0, -13.0);
  Point expected_point = Point(46.5, -12.37); 
  int expected_id = 76;

  auto result = ctrl.point_solver_.update_closest_point(path_msg_ptr, rear_axis);
  
  EXPECT_EQ(result.first.x_, expected_point.x_);
  EXPECT_EQ(result.first.y_, expected_point.y_);
  EXPECT_EQ(result.second, expected_id);
}

/**
 * @brief Test Point Solver - update_lookahead_point()
 */

TEST(PointSolverTests, Test_update_lookahead_point_1) {
  custom_interfaces::msg::PathPointArray path_msg = create_path_msg("track1");

  // Convert path_msg to ConstSharedPtr
  const custom_interfaces::msg::PathPointArray::ConstSharedPtr path_msg_ptr =
      std::make_shared<const custom_interfaces::msg::PathPointArray>(path_msg);
   

  
  double ld = 3;
  double k=3;
  double ld_margin = 0.1;

    Control ctrl;

    ctrl.k_=k;
    ctrl.ld_margin_=ld_margin;
  Point rear_axis = Point(47.0, -13.0);
  //Point cg = Point(47.80, -12.43);
  //Point closest_point = Point(46.5, -12.37);
  int closest_point_id = 76;
  Point expected_point = Point(48.75, -10.25);

  auto [result_point, result_velocity, result_error] = ctrl.point_solver_.update_lookahead_point(path_msg_ptr, rear_axis, closest_point_id, ld, ld_margin);
  
  EXPECT_EQ(result_point.x_, expected_point.x_);
  EXPECT_EQ(result_point.y_, expected_point.y_);
  EXPECT_FALSE(result_error);
}