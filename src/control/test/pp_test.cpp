#include "pure_pursuit/pp.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
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

/**
 * @brief Test PurePursuit - update_closest_point()
 */
TEST(PurePursuitTests, Test_update_closest_point_1) {
  std::ifstream trackFile("../../src/control/test/assets/track1.csv");
  ASSERT_TRUE(trackFile.is_open()) << "Failed to open track file";

  custom_interfaces::msg::PathPointArray path_msg;

  std::string line;

  std::getline(trackFile, line); // Skip the first line

  while (std::getline(trackFile, line)) {
    std::istringstream iss(line);
    std::string x, y, v;
    if (std::getline(iss, x, ',') && std::getline(iss, y, ',') && std::getline(iss, v)) {
      // Create a PathPoint and add it to the PathPointArray
      custom_interfaces::msg::PathPoint point;
      point.x = std::stod(x); //string to double
      point.y = std::stod(y);
      point.v = std::stod(v);
      path_msg.pathpoint_array.push_back(point);
    }
  }
  trackFile.close();

  // Convert path_msg to ConstSharedPtr
  const custom_interfaces::msg::PathPointArray::ConstSharedPtr path_msg_ptr =
      std::make_shared<const custom_interfaces::msg::PathPointArray>(path_msg);


  double k = 0.5;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  pp.vehicle_pose_.rear_axis_ = Point(47.0, -13.0);
  Point expected_point = Point(46.5, -12.37); 
  int expected_id = 76;

  auto result = pp.update_closest_point(path_msg_ptr);
  
  EXPECT_EQ(result.first.x_, expected_point.x_);
  EXPECT_EQ(result.first.y_, expected_point.y_);
  EXPECT_EQ(result.second, expected_id);
}

/**
 * @brief Test PurePursuit - update_lookahead_point()
 */

TEST(PurePursuitTests, Test_update_lookahead_point_1) {
  std::ifstream trackFile("../../src/control/test/assets/track1.csv");
  ASSERT_TRUE(trackFile.is_open()) << "Failed to open track file";

  custom_interfaces::msg::PathPointArray path_msg;

  std::string line;

  std::getline(trackFile, line); // Skip the first line

  while (std::getline(trackFile, line)) {
    std::istringstream iss(line);
    std::string x, y, v;
    if (std::getline(iss, x, ',') && std::getline(iss, y, ',') && std::getline(iss, v)) {
      custom_interfaces::msg::PathPoint point;
      point.x = std::stod(x); //string to double
      point.y = std::stod(y);
      point.v = std::stod(v);
      path_msg.pathpoint_array.push_back(point);
    }
  }
  trackFile.close();

  // Convert path_msg to ConstSharedPtr
  const custom_interfaces::msg::PathPointArray::ConstSharedPtr path_msg_ptr =
      std::make_shared<const custom_interfaces::msg::PathPointArray>(path_msg);
   

  double k = 3;
  double ld = 3;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);

  pp.vehicle_pose_.rear_axis_ = Point(47.0, -13.0);
  pp.vehicle_pose_.cg_ = Point(47.80, -12.43);
  Point closest_point = Point(46.5, -12.37);
  int closest_point_id = 76;
  Point expected_point = Point(48.75, -10.25);

  auto result = pp.update_lookahead_point(path_msg_ptr, closest_point, closest_point_id, ld, ld_margin);
  
  EXPECT_EQ(result.first.x_, expected_point.x_);
  EXPECT_EQ(result.first.y_, expected_point.y_);
  EXPECT_FALSE(result.second);
}


/**
 * @brief Test PurePursuit - update_steering_angle()
 */

TEST(PurePursuitTests, Test_update_steering_angle_1) {
  std::ifstream trackFile("../../src/control/test/assets/track1.csv");
  ASSERT_TRUE(trackFile.is_open()) << "Failed to open track file";

  custom_interfaces::msg::PathPointArray path_msg;

  std::string line;

  std::getline(trackFile, line); // Skip the first line

  while (std::getline(trackFile, line)) {
    std::istringstream iss(line);
    std::string x, y, v;
    if (std::getline(iss, x, ',') && std::getline(iss, y, ',') && std::getline(iss, v)) {
      custom_interfaces::msg::PathPoint point;
      point.x = std::stod(x); //string to double
      point.y = std::stod(y);
      point.v = std::stod(v);
      path_msg.pathpoint_array.push_back(point);
    }
  }
  trackFile.close();

  // Convert path_msg to ConstSharedPtr
  const custom_interfaces::msg::PathPointArray::ConstSharedPtr path_msg_ptr =
      std::make_shared<const custom_interfaces::msg::PathPointArray>(path_msg);
  
  custom_interfaces::msg::Pose pose_msg;
  pose_msg.position.x = 47.80;
  pose_msg.position.y = -12.43;
  pose_msg.theta=0.610865; //35 degrees
  pose_msg.velocity=1.0;

  const custom_interfaces::msg::Pose::ConstSharedPtr pose_msg_ptr =
      std::make_shared<const custom_interfaces::msg::Pose>(pose_msg);

      
  //DIST_CG_2_REAR_AXIS = 0.9822932352409;

  double k = 3;
  double ld_margin = 0.1;
  PurePursuit pp(k, ld_margin);
  double steering_cmd = pp.update_steering_angle(path_msg_ptr,pose_msg_ptr);
  EXPECT_NEAR(0.33, steering_cmd, 0.01); // 0.01 rad is equal to 0.57 degrees
  /*
  alpha = 0.3833 (rad)
  L = 1 || 0.9822932352409
  Closest Point (46.5, -12.37)
  Vehicle CG (47.80,-12.43)
  Vehicle Rear Axis (47 , -13)
  Lookahead Point (48.75, -10.25)
  Lookahead Distance (3 +/- 10%)
  actual ld = 3.2511690205217
  */
  }