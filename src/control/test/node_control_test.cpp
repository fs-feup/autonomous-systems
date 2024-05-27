#include "node_/node_control.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "test/include/utils.hpp"


/**
 * @brief Test Node Control - update_steering_angle()
 * THIS IS A MORE COMPLETE TEST TO TEST THE PURE PURSUIT ALGORITHM 
 * ONLY WORKS IF :
 * -> THE FUNCTION IS CHANGED TO RETURN THE STEERING ANGLE
 * -> COMMENT ALL PUBLISHERS IN orchestrator_callback() 
 * -> UNCOMMENT THIS TEST
 */

/*
TEST(NodeControlTests, Test_update_steering_angle_1) {
  custom_interfaces::msg::PathPointArray path_msg = create_path_msg("track1");

  // Convert path_msg to ConstSharedPtr
  const custom_interfaces::msg::PathPointArray::ConstSharedPtr path_msg_ptr =
      std::make_shared<const custom_interfaces::msg::PathPointArray>(path_msg);

  custom_interfaces::msg::Pose pose_msg;
  pose_msg.position.x = 47.80;
  pose_msg.position.y = -12.43;
  pose_msg.theta = 0.610865;  // 35 degrees
  pose_msg.velocity = 1.0;

  const custom_interfaces::msg::Pose::ConstSharedPtr pose_msg_ptr =
      std::make_shared<const custom_interfaces::msg::Pose>(pose_msg);

  //double ld = 3;
  double k = 3;
  double ld_margin = 0.1;

  Control ctrl;
  ctrl.k_ = k;
  ctrl.ld_margin_ = ld_margin;
  
  double steering_cmd = ctrl.orchestrator_callback(path_msg_ptr, pose_msg_ptr);
  EXPECT_NEAR(0.33, steering_cmd, 0.01);  // 0.01 rad is equal to 0.57 degrees
  
}
*/
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
