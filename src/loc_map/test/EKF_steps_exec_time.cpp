#include <fstream>  //to write file

#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
// microsegundos - /100

#include <fstream>  //to write file

#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test/fixture.hpp"

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_10) {
  // create motion update and state with adequate workload

  motion_update_test->translational_velocity = 1.58113883;
  motion_update_test->translational_velocity_x = 1.5;
  motion_update_test->translational_velocity_y = 0.5;
  motion_update_test->rotational_velocity = 6.0;
  motion_update_test->steering_angle = 2.0;
  motion_update_test->last_update = std::chrono::high_resolution_clock::now();
  ekf_test->set_X(23);
  ekf_test->set_P(23);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  // necessary?
  ekf_test->set_X_y(3, -1.637208342552185);
  ekf_test->set_X_y(4, 14.400202751159668);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(5, -2.216218948364258);
  ekf_test->set_X_y(6, 11.487205505371094);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(7, -3.867227792739868);
  ekf_test->set_X_y(8, 9.018211364746094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(9, -6.336233615875244);
  ekf_test->set_X_y(10, 7.367220401763916);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(11, -9.250235557556152);
  ekf_test->set_X_y(12, 6.788230895996094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(13, 16.861791610717773);
  ekf_test->set_X_y(14, 14.40013599395752);
  ekf_test->push_to_colors(colors::Color::yellow);

  ekf_test->set_X_y(15, 16.28278160095215);
  ekf_test->set_X_y(16, 11.487138748168945);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(17, 14.6317720413208);
  ekf_test->set_X_y(18, 9.018143653869629);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(19, 12.162766456604004);
  ekf_test->set_X_y(20, 7.367153644561768);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(21, 9.249764442443848);
  ekf_test->set_X_y(22, 6.788164138793945);
  ekf_test->push_to_colors(colors::Color::yellow);
  //\necessary?

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step(*motion_update_test);
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF pred step, 10";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_50) {
  // create motion update and state with adequate workload

  motion_update_test->translational_velocity = 1.58113883;
  motion_update_test->translational_velocity_x = 1.5;
  motion_update_test->translational_velocity_y = 0.5;
  motion_update_test->rotational_velocity = 6.0;
  motion_update_test->steering_angle = 2.0;
  motion_update_test->last_update = std::chrono::high_resolution_clock::now();
  ekf_test->set_X(103);
  ekf_test->set_P(103);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(102);

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step(*motion_update_test);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF pred step, 50";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_100) {
  // create motion update and state with adequate workload(size)

  ekf_test->set_X(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(202);

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step(*motion_update_test);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF pred step, 100";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_10) {
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();

  ekf_test->set_X(23);
  ekf_test->set_P(23);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  // necessary?
  ekf_test->set_X_y(3, -1.637208342552185);
  ekf_test->set_X_y(4, 14.400202751159668);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(5, -2.216218948364258);
  ekf_test->set_X_y(6, 11.487205505371094);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(7, -3.867227792739868);
  ekf_test->set_X_y(8, 9.018211364746094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(9, -6.336233615875244);
  ekf_test->set_X_y(10, 7.367220401763916);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(11, -9.250235557556152);
  ekf_test->set_X_y(12, 6.788230895996094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(13, 16.861791610717773);
  ekf_test->set_X_y(14, 14.40013599395752);
  ekf_test->push_to_colors(colors::Color::yellow);

  ekf_test->set_X_y(15, 16.28278160095215);
  ekf_test->set_X_y(16, 11.487138748168945);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(17, 14.6317720413208);
  ekf_test->set_X_y(18, 9.018143653869629);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(19, 12.162766456604004);
  ekf_test->set_X_y(20, 7.367153644561768);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(21, 9.249764442443848);
  ekf_test->set_X_y(22, 6.788164138793945);
  ekf_test->push_to_colors(colors::Color::yellow);
  //\necessary?

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->correction_step(coneMap);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF Correction Step, 10 and 10 From \"perception\"";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_50) {
  // create cone map , maybe state
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();

  ekf_test->set_X(103);
  ekf_test->set_P(103);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  // necessary?
  fill_X(102);

  start_time = std::chrono::high_resolution_clock::now();

  ekf_test->correction_step(coneMap);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF Correction Step, 50 and 10 From \"perception\"";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_100) {
  // create cone map , maybe state
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();
  ekf_test->set_X(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  fill_X(202);

  start_time = std::chrono::high_resolution_clock::now();

  ekf_test->correction_step(coneMap);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF Correction Step, 100 and 10 From \"perception\"";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}