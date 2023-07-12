#include "gtest/gtest.h"
#include "kalman_filter/motion_models.hpp"
#include "loc_map/lm_subscriber.hpp"

/* ---------------------- Motion Model -------------------------------------*/

TEST(MOTION_MODEL, NOISE_MATRIX_SHAPE_TEST) {
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  R(0, 0) = 0.1;
  R(1, 1) = 0.1;
  R(2, 2) = 0.1;

  MotionModel* motion_model = new NormalVelocityModel(R);
  Eigen::MatrixXf noise_matrix = motion_model->get_process_noise_covariance_matrix(10);
  EXPECT_EQ(noise_matrix.rows(), 10);
  EXPECT_EQ(noise_matrix.cols(), 10);
  EXPECT_NEAR(noise_matrix(0, 0), 0.1, 0.0001);
  EXPECT_NEAR(noise_matrix(1, 1), 0.1, 0.0001);
  EXPECT_NEAR(noise_matrix(2, 2), 0.1, 0.0001);
}

/* ---------------------- Normal Velocity Model ---------------------------*/

TEST(NORMAL_VELOCITY_MODEL, STANDING_STILL_TEST) {
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  NormalVelocityModel motion_model = NormalVelocityModel(R);
  MotionPredictionData prediction_data = {0, 0, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf G =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = G * Eigen::MatrixXf::Zero(10, 10) * G.transpose();
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
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  NormalVelocityModel motion_model = NormalVelocityModel(R);
  MotionPredictionData prediction_data = {1, 0, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf G =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance =
      G * Eigen::MatrixXf::Ones(10, 10) *
      G.transpose();                    // Covariance with ones to check if it is modified
  EXPECT_DOUBLE_EQ(new_state(0), 1.0);  // x
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);  // y
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);  // theta
  for (int i = 0; i < 10; i++) {        // Covariance
    for (int j = 0; j < 10; j++) {
      if (i == 1 || j == 1)  // Only y is affected because of the Jacobian
        EXPECT_NE(new_covariance(i, j), 1.0);
      else
        EXPECT_DOUBLE_EQ(new_covariance(i, j), 1.0);
    }
  }
}

TEST(NORMAL_VELOCITY_MODEL, LINEAR_VELOCITY_CURVE_TEST) {
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  NormalVelocityModel motion_model = NormalVelocityModel(R);

  // Moving in a curve with linear acceleration
  MotionPredictionData prediction_data = {1, 0, 0, M_PI / 180};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf G =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance =
      G * Eigen::MatrixXf::Ones(10, 10) *
      G.transpose();                      // Covariance with ones to check if it is modified
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
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  NormalVelocityModel motion_model = NormalVelocityModel(R);
  MotionPredictionData prediction_data = {0, 0, 0, 0};
  Eigen::VectorXf temp_state;
  Eigen::VectorXf new_state = Eigen::VectorXf::Zero(10);
  double radius = 12.7324;
  new_state(0) = 0;
  new_state(1) = -radius;  // Radius of the circle
  Eigen::MatrixXf G =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = Eigen::MatrixXf::Ones(10, 10);
  for (int i = 0; i < 1000; i++) {
    prediction_data.translational_velocity = 10;
    prediction_data.rotational_velocity = M_PI / 4;
    temp_state = new_state;
    new_state = motion_model.predict_expected_state(new_state, prediction_data, 0.1);
    G = motion_model.get_motion_to_state_matrix(new_state, prediction_data, 0.1);
    new_covariance = G * new_covariance * G.transpose();
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
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  ImuVelocityModel motion_model = ImuVelocityModel(R);
  MotionPredictionData prediction_data = {0, 0, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf G =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = G * Eigen::MatrixXf::Zero(10, 10) * G.transpose();
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
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  ImuVelocityModel motion_model = ImuVelocityModel(R);
  MotionPredictionData prediction_data = {0, 1, 0, 0};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf G =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = G * Eigen::MatrixXf::Ones(10, 10) * G.transpose();
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
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  ImuVelocityModel motion_model = ImuVelocityModel(R);
  MotionPredictionData prediction_data = {0, 0.3, 0.7, M_PI / 16};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 0.1);
  Eigen::MatrixXf G =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance =
      G * Eigen::MatrixXf::Ones(10, 10) *
      G.transpose();                         // Covariance with ones to check if it is modified
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
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  ImuVelocityModel motion_model = ImuVelocityModel(R);
  MotionPredictionData prediction_data = {0, 1, 0, 0};
  Eigen::VectorXf temp_state;
  Eigen::VectorXf new_state = Eigen::VectorXf::Zero(10);
  Eigen::MatrixXf G;
  Eigen::MatrixXf new_covariance = Eigen::MatrixXf::Ones(10, 10);
  for (int i = 0; i < 1000; i++) {
    prediction_data.translational_velocity_x = 10 * i;
    prediction_data.translational_velocity_y = 5 * i;
    prediction_data.rotational_velocity = 5 * i;
    temp_state = new_state;
    new_state = motion_model.predict_expected_state(new_state, prediction_data, 0.1);
    G = motion_model.get_motion_to_state_matrix(new_state, prediction_data, 0.1);
    new_covariance = G * new_covariance * G.transpose();
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

/* ----------------------- ODOMETRY MODEL -------------------------*/

/**
 * @brief Tests the conversion from wheel revolutions
 * to velocities using the bycicle model
 *
 */
TEST(ODOMETRY_SUBSCRIBER, CONVERSION_TEST) {
  // Straight Line
  MotionPredictionData prediction_data_from_odometry;
  double lb_speed = 60;
  double rb_speed = 60;
  double lf_speed = 60;
  double rf_speed = 60;
  double steering_angle = 0;
  MotionUpdate velocity_data = LMSubscriber::odometry_to_velocities_transform(
      lb_speed, rb_speed, lf_speed, rf_speed, steering_angle);
  EXPECT_NEAR(velocity_data.translational_velocity, 1.5708, 0.0001);
  EXPECT_DOUBLE_EQ(velocity_data.rotational_velocity, 0);
  EXPECT_DOUBLE_EQ(velocity_data.translational_velocity_x, 0);
  EXPECT_DOUBLE_EQ(velocity_data.translational_velocity_y, 0);

  // Curving left
  lb_speed = 60;
  rb_speed = 60;
  lf_speed = 60;
  rf_speed = 60;
  steering_angle = M_PI / 8;
  velocity_data = LMSubscriber::odometry_to_velocities_transform(lb_speed, rb_speed, lf_speed,
                                                                 rf_speed, steering_angle);
  EXPECT_GE(velocity_data.translational_velocity, 1.5708);
  EXPECT_LE(velocity_data.translational_velocity, 1.5708 * 2);
  EXPECT_LE(velocity_data.rotational_velocity, M_PI);
  EXPECT_GE(velocity_data.rotational_velocity, M_PI / 8);
  EXPECT_DOUBLE_EQ(velocity_data.translational_velocity_x, 0);
  EXPECT_DOUBLE_EQ(velocity_data.translational_velocity_y, 0);

  // Curving right
  lb_speed = 60;
  rb_speed = 60;
  lf_speed = 60;
  rf_speed = 60;
  steering_angle = -M_PI / 8;
  MotionUpdate velocity_data_2 = LMSubscriber::odometry_to_velocities_transform(
      lb_speed, rb_speed, lf_speed, rf_speed, steering_angle);
  EXPECT_GE(velocity_data_2.translational_velocity, 1.5708);
  EXPECT_LE(velocity_data_2.translational_velocity, 1.5708 * 2);
  EXPECT_GE(velocity_data_2.rotational_velocity, -M_PI);
  EXPECT_LE(velocity_data_2.rotational_velocity, -M_PI / 8);
  EXPECT_DOUBLE_EQ(velocity_data_2.translational_velocity_x, 0);
  EXPECT_DOUBLE_EQ(velocity_data_2.translational_velocity_y, 0);
  EXPECT_DOUBLE_EQ(velocity_data_2.translational_velocity, velocity_data.translational_velocity);
  EXPECT_DOUBLE_EQ(velocity_data_2.rotational_velocity, -velocity_data.rotational_velocity);
}