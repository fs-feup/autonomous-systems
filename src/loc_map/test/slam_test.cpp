#include "gtest/gtest.h"

#include "kalman_filter/ekf.hpp"
#include "loc_map/lm_subscriber.hpp"
#include "loc_map/data_structures.hpp"

TEST(SLAM_TEST_SUITE, NORMAL_VELOCITY_MODEL_TEST) {

  // VehicleState* vehicle_state = new VehicleState();
  // ImuUpdate *imu_update = new ImuUpdate();
  // Map* track_map = new Map();

  // rclcpp::init(0, nullptr);

  // auto subscriber = std::make_shared<LMSubscriber>(track_map, imu_update);
  
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(subscriber);
  // while (rclcpp::ok()) {
  //   executor.spin_some();
  // }

  // rclcpp::shutdown();

  // Standing still
  NormalVelocityModel motion_model = NormalVelocityModel();
  MotionPredictionData prediction_data = {
      0,
      0,
      0,
      0
  };
  Eigen::VectorXf new_state = motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(200), prediction_data, 1.0);
  EXPECT_DOUBLE_EQ(new_state(0), 0.0);
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);
  EXPECT_DOUBLE_EQ(new_state(3), 0.0);

  // Moving forward linearly
  prediction_data = {
      1,
      0,
      0,
      0
  };
  new_state = motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(200), prediction_data, 1.0);
  EXPECT_DOUBLE_EQ(new_state(0), 0.0); // x
  EXPECT_DOUBLE_EQ(new_state(1), 1.0); // y
  EXPECT_DOUBLE_EQ(new_state(2), 0.0); // theta

//   // Moving in a curve with linear acceleration
//   prediction_data = {
//       1,
//       0,
//       0,
//       1
//   };
//   new_state = motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(200), prediction_data, 1.0);
//   EXPECT_LE(new_state(0), -0.1); // x
//   EXPECT_GE(new_state(0), -0.3); // x
//   EXPECT_LE(new_state(1), 0.9); // y
//   EXPECT_GE(new_state(1), 0.7); // y
//   EXPECT_EQ(new_state(2), 1.0); // theta
}

TEST(SLAM_TEST_SUITE, IMU_VELOCITY_MODEL_TEST) {

  // Standing still
  ImuVelocityModel motion_model = ImuVelocityModel();
  MotionPredictionData prediction_data = {
      0,
      0,
      0,
      0
  };
  Eigen::VectorXf new_state = motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(200), prediction_data, 1.0);
  EXPECT_DOUBLE_EQ(new_state(0), 0.0);
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);
  EXPECT_DOUBLE_EQ(new_state(3), 0.0);

  // Moving forward linearly
  prediction_data = {
      0,
      0,
      1,
      0
  };
  new_state = motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(200), prediction_data, 1.0);
  EXPECT_DOUBLE_EQ(new_state(0), 0.0); // x
  EXPECT_DOUBLE_EQ(new_state(1), 1.0); // y
  EXPECT_DOUBLE_EQ(new_state(2), 0.0); // theta

  // Moving in a curve with linear acceleration
  prediction_data = {
      0,
      0.3,
      0.7,
      1
  };
  new_state = motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(200), prediction_data, 0.1);
  EXPECT_LT(new_state(0), 0.030001); // x
  EXPECT_LT(new_state(1), 0.070001); // y
  EXPECT_LT(new_state(2), 0.10001); // theta
  EXPECT_GT(new_state(0), 0.029999); // x
  EXPECT_GT(new_state(1), 0.069999); // y
  EXPECT_GT(new_state(2), 0.09999); // theta

  // Complex movement
  prediction_data = {
    0,
    1,
    0,
    0
  };
  Eigen::VectorXf temp_state = Eigen::VectorXf::Zero(200);
  for (int i = 0; i < 1000; i++) {
    prediction_data.translational_velocity_x = 10 * i; 
    prediction_data.translational_velocity_y = 5 * i;
    prediction_data.rotational_velocity = 5 * i;
    temp_state = new_state;
    new_state = motion_model.motion_model_expected_state(new_state, prediction_data, 0.1);
    // Max 0.01 percent calculation error
    EXPECT_LE(new_state(0), temp_state(0) + i + 0.0001 * temp_state(0)); // x
    EXPECT_LE(new_state(1), temp_state(1) + 0.5 * i + 0.0001 * temp_state(1)); // y
    EXPECT_GE(new_state(0), temp_state(0) + i - 0.0001 * temp_state(0)); // x
    EXPECT_GE(new_state(1), temp_state(1) + 0.5 * i - 0.0001 * temp_state(1)); // y
    EXPECT_LT(new_state(2), 360); 
  }
}