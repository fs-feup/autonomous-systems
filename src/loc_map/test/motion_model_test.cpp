#include "gtest/gtest.h"
#include "kalman_filter/motion_models.hpp"
#include "loc_map/data_structures.hpp"
#include "loc_map/lm_subscriber.hpp"

/* ---------------------- Normal Velocity Model ---------------------------*/

TEST(NORMAL_VELOCITY_MODEL, STANDING_STILL_TEST) {
  NormalVelocityModel motion_model = NormalVelocityModel();
  MotionPredictionData prediction_data = {0, 0, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = motion_model.motion_model_covariance_matrix(
      Eigen::MatrixXf::Zero(10, 10), Eigen::MatrixXf::Zero(10, 10), prediction_data, 1.0);
  EXPECT_DOUBLE_EQ(new_state(0), 0.0);
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      EXPECT_DOUBLE_EQ(new_covariance(i, j), 0.0);
    }
  }
}

TEST(NORMAL_VELOCITY_MODEL, LINEAR_FORWARD_MOVEMENT_TEST) {
  NormalVelocityModel motion_model = NormalVelocityModel();
  MotionPredictionData prediction_data = {1, 0, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = motion_model.motion_model_covariance_matrix(
      Eigen::MatrixXf::Ones(10, 10), Eigen::MatrixXf::Zero(10, 10), prediction_data,
      1.0);                             // Covariance with ones to check if it is modified
  EXPECT_DOUBLE_EQ(new_state(0), 1.0);  // x
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);  // y
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);  // theta
  for (int i = 0; i < 10; i++) {        // Covariance
    for (int j = 0; j < 10; j++) {
      if (i < 2 || j < 2)  // Only x and y are affected because of the Jacobian
        EXPECT_NE(new_covariance(i, j), 1.0);
      else
        EXPECT_DOUBLE_EQ(new_covariance(i, j), 1.0);
    }
  }
}

TEST(NORMAL_VELOCITY_MODEL, LINEAR_VELOCITY_CURVE_TEST) {
  NormalVelocityModel motion_model = NormalVelocityModel();

  // Moving in a curve with linear acceleration
  MotionPredictionData prediction_data = {1, 0, 0, M_PI / 180};
  Eigen::VectorXf new_state =
      motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = motion_model.motion_model_covariance_matrix(
      Eigen::MatrixXf::Ones(10, 10), Eigen::MatrixXf::Zero(10, 10), prediction_data,
      1.0);                               // Covariance with ones to check if it is modified
  EXPECT_NEAR(new_state(0), 0.99, 0.05);  // x
  EXPECT_NEAR(new_state(1), 0.01, 0.05);  // y
  EXPECT_NEAR(new_state(2), M_PI / 180, 0.00001);  // theta
  for (int i = 0; i < 10; i++) {                   // Covariance
    for (int j = 0; j < 10; j++) {
      if (i < 2 || j < 2)  // Only x and y are affected because of the Jacobian
        EXPECT_NE(new_covariance(i, j), 1.0);
      else
        EXPECT_DOUBLE_EQ(new_covariance(i, j), 1.0);
    }
  }
}

TEST(NORMAL_VELOCITY_MODEL, CIRCULAR_MOVEMENT_TEST) {
  NormalVelocityModel motion_model = NormalVelocityModel();
  MotionPredictionData prediction_data = {0, 0, 0, 0};
  Eigen::VectorXf temp_state;
  Eigen::VectorXf new_state = Eigen::VectorXf::Zero(10);
  double radius = 12.7324;
  new_state(0) = 0;
  new_state(1) = -radius;  // Radius of the circle
  Eigen::MatrixXf new_covariance = Eigen::MatrixXf::Ones(10, 10);
  for (int i = 0; i < 1000; i++) {
    prediction_data.translational_velocity = 10;
    prediction_data.rotational_velocity = M_PI / 4;
    temp_state = new_state;
    new_state = motion_model.motion_model_expected_state(new_state, prediction_data, 0.1);
    new_covariance = motion_model.motion_model_covariance_matrix(
        new_covariance, Eigen::MatrixXf::Zero(10, 10), prediction_data, 0.1);
    // Max 0.01 percent calculation error
    EXPECT_LT(new_state(0), radius + 0.1);  // x
    EXPECT_LT(new_state(1), radius + 0.1);  // y
    EXPECT_NEAR(new_state(0) * new_state(0) + new_state(1) * new_state(1), radius * radius,
                0.0001 * radius * radius);  // Radius
    EXPECT_LT(new_state(2), 2 * M_PI);      // theta

    for (int i = 0; i < 10; i++) {  // Covariance
      for (int j = 0; j < 10; j++) {
        if (i < 2 || j < 2)  // Only x and y are affected because of the Jacobian
          EXPECT_NE(new_covariance(i, j), 1.0);
        else
          EXPECT_DOUBLE_EQ(new_covariance(i, j), 1.0);
      }
    }
  }
}

/* ----------------------- IMU VELOCITY MODEL -------------------------*/

TEST(IMU_VELOCITY_MODEL, STANDING_STILL_TEST) {
  ImuVelocityModel motion_model = ImuVelocityModel();
  MotionPredictionData prediction_data = {0, 0, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = motion_model.motion_model_covariance_matrix(
      Eigen::MatrixXf::Zero(10, 10), Eigen::MatrixXf::Zero(10, 10), prediction_data, 1.0);
  EXPECT_DOUBLE_EQ(new_state(0), 0.0);
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      EXPECT_DOUBLE_EQ(new_covariance(i, j), 0.0);
    }
  }
}

TEST(IMU_VELOCITY_MODEL, LINEAR_FORWARD_MOVEMENT_TEST) {
  ImuVelocityModel motion_model = ImuVelocityModel();
  MotionPredictionData prediction_data = {0, 1, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = motion_model.motion_model_covariance_matrix(
      Eigen::MatrixXf::Ones(10, 10), Eigen::MatrixXf::Zero(10, 10), prediction_data, 1.0);
  EXPECT_DOUBLE_EQ(new_state(0), 1.0);  // x
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);  // y
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);  // theta
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      EXPECT_DOUBLE_EQ(new_covariance(i, j), 1.0);
    }
  }
}

TEST(IMU_VELOCITY_MODEL, LINEAR_VELOCITY_CURVE_TEST) {
  ImuVelocityModel motion_model = ImuVelocityModel();
  MotionPredictionData prediction_data = {0, 0.3, 0.7, M_PI / 16};
  Eigen::VectorXf new_state =
      motion_model.motion_model_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 0.1);
  Eigen::MatrixXf new_covariance = motion_model.motion_model_covariance_matrix(
      Eigen::MatrixXf::Ones(10, 10), Eigen::MatrixXf::Zero(10, 10), prediction_data,
      1.0);                                  // Covariance with ones to check if it is modified
  EXPECT_NEAR(new_state(0), 0.03, 0.00001);  // x
  EXPECT_NEAR(new_state(1), 0.07, 0.00001);  // y
  EXPECT_NEAR(new_state(2), M_PI / 160, 0.00001);  // theta
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      EXPECT_DOUBLE_EQ(new_covariance(i, j), 1.0);
    }
  }
}

TEST(IMU_VELOCITY_MODEL, COMPLEX_MOVEMENT_TEST) {
  ImuVelocityModel motion_model = ImuVelocityModel();
  MotionPredictionData prediction_data = {0, 1, 0, 0};
  Eigen::VectorXf temp_state;
  Eigen::VectorXf new_state = Eigen::VectorXf::Zero(10);
  Eigen::MatrixXf new_covariance = Eigen::MatrixXf::Ones(10, 10);
  for (int i = 0; i < 1000; i++) {
    prediction_data.translational_velocity_x = 10 * i;
    prediction_data.translational_velocity_y = 5 * i;
    prediction_data.rotational_velocity = 5 * i;
    temp_state = new_state;
    new_state = motion_model.motion_model_expected_state(new_state, prediction_data, 0.1);
    new_covariance = motion_model.motion_model_covariance_matrix(
        new_covariance, Eigen::MatrixXf::Zero(10, 10), prediction_data, 0.1);
    // Max 0.01 percent calculation error
    EXPECT_NEAR(new_state(0), temp_state(0) + i, 0.0001 * temp_state(0));        // x
    EXPECT_NEAR(new_state(1), temp_state(1) + 0.5 * i, 0.0001 * temp_state(1));  // y
    EXPECT_LT(new_state(2), 2 * M_PI);                                           // theta

    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < 10; j++) {
        EXPECT_DOUBLE_EQ(new_covariance(i, j), 1.0);
      }
    }
  }
}