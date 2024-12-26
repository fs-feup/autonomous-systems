#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "bicycle_model.hpp"
#include "gtest/gtest.h"
#include "kalman_filter/motion_models.hpp"
#include "ros_node/se_node.hpp"

/* ---------------------- Motion Model -------------------------------------*/

TEST(MOTION_MODEL, NOISE_MATRIX_SHAPE_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  r_matrix(0, 0) = static_cast<float>(0.1);
  r_matrix(1, 1) = static_cast<float>(0.1);
  r_matrix(2, 2) = static_cast<float>(0.1);
  r_matrix(3, 3) = static_cast<float>(0.1);
  r_matrix(4, 4) = static_cast<float>(0.1);
  r_matrix(5, 5) = static_cast<float>(0.1);

  std::shared_ptr<MotionModel> motion_model = std::make_shared<NormalVelocityModel>(r_matrix);
  Eigen::MatrixXf noise_matrix = motion_model->get_process_noise_covariance_matrix(10);
  EXPECT_EQ(noise_matrix.rows(), 10);
  EXPECT_EQ(noise_matrix.cols(), 10);
  EXPECT_NEAR(noise_matrix(0, 0), 0.1, 0.0001);
  EXPECT_NEAR(noise_matrix(1, 1), 0.1, 0.0001);
  EXPECT_NEAR(noise_matrix(2, 2), 0.1, 0.0001);
  EXPECT_NEAR(noise_matrix(3, 3), 0.1, 0.0001);
  EXPECT_NEAR(noise_matrix(4, 4), 0.1, 0.0001);
}

/* ---------------------- Normal Velocity Model ---------------------------*/

TEST(NORMAL_VELOCITY_MODEL, STANDING_STILL_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(5, 5);
  NormalVelocityModel motion_model = NormalVelocityModel(r_matrix);
  MotionUpdate prediction_data = {0, 0, 0, 0, 0, rclcpp::Clock().now()};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = g_matrix * Eigen::MatrixXf::Zero(10, 10) * g_matrix.transpose();
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
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  NormalVelocityModel motion_model = NormalVelocityModel(r_matrix);
  MotionUpdate prediction_data = {1, 0, 0, 0, 0, rclcpp::Clock().now()};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);

  Eigen::MatrixXf new_covariance =
      g_matrix * Eigen::MatrixXf::Ones(10, 10) *
      g_matrix.transpose();             // Covariance with ones to check if it is modified
  EXPECT_DOUBLE_EQ(new_state(0), 1.0);  // x
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);  // y
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);  // theta
  for (int i = 0; i < 10; i++) {        // Covariance
    for (int j = 0; j < 10; j++) {
      if (i == 1 || j == 1) {
        EXPECT_NE(new_covariance(i, j), 1.0);
      }  // Only y is affected because of the Jacobian
    }
  }
}

TEST(NORMAL_VELOCITY_MODEL, LINEAR_VELOCITY_CURVE_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(5, 5);
  NormalVelocityModel motion_model = NormalVelocityModel(r_matrix);

  // Moving in a curve with linear acceleration
  MotionUpdate prediction_data = {1, 0, 0, M_PI / 180, 0, rclcpp::Clock().now()};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance =
      g_matrix * Eigen::MatrixXf::Ones(10, 10) *
      g_matrix.transpose();               // Covariance with ones to check if it is modified
  EXPECT_NEAR(new_state(0), 0.99, 0.05);  // x
  EXPECT_NEAR(new_state(1), 0.01, 0.05);  // y
  EXPECT_NEAR(new_state(2), M_PI / 180, 0.00001);  // theta
}

TEST(NORMAL_VELOCITY_MODEL, AUTONOMOUS_DEMO_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  Eigen::VectorXf temp_state = Eigen::VectorXf::Zero(6);
  NormalVelocityModel motion_model = NormalVelocityModel(r_matrix);
  MotionUpdate prediction_data = {2.5, 0, 0, 0.001, 0, rclcpp::Clock().now()};
  for (unsigned int i = 0; i < 4; i++) {
    temp_state = motion_model.predict_expected_state(temp_state, prediction_data, 1);
    EXPECT_NEAR(temp_state(0), 2.5 * (i + 1), 0.01 * (i + 1));
  }
  prediction_data = {0, 0, 0, 0.001, 0, rclcpp::Clock().now()};
  for (unsigned int i = 0; i < 4; i++) {
    temp_state = motion_model.predict_expected_state(temp_state, prediction_data, 1);
    EXPECT_NEAR(temp_state(0), 10, 0.05);
  }
  prediction_data = {2.5, 0, 0, 0.001, 0, rclcpp::Clock().now()};
  for (unsigned int i = 0; i < 4; i++) {
    temp_state = motion_model.predict_expected_state(temp_state, prediction_data, 1);
    EXPECT_NEAR(temp_state(0), 10 + 2.5 * (i + 1), 0.01 * (i + 1));
  }
}

TEST(NORMAL_VELOCITY_MODEL, CIRCULAR_MOVEMENT_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  NormalVelocityModel motion_model = NormalVelocityModel(r_matrix);
  MotionUpdate prediction_data = {0, 0, 0, 0, 0, rclcpp::Clock().now()};
  Eigen::VectorXf temp_state;
  Eigen::VectorXf new_state = Eigen::VectorXf::Zero(10);
  float radius = static_cast<float>(12.7324);
  new_state(0) = 0;
  new_state(1) = -radius;  // Radius of the circle
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = Eigen::MatrixXf::Ones(10, 10);
  for (int i = 0; i < 1000; i++) {
    prediction_data.translational_velocity = 10;
    prediction_data.rotational_velocity = M_PI / 4;
    temp_state = new_state;
    new_state = motion_model.predict_expected_state(new_state, prediction_data, 0.1);
    g_matrix = motion_model.get_motion_to_state_matrix(new_state, prediction_data, 0.1);
    new_covariance = g_matrix * new_covariance * g_matrix.transpose();
    // Max 0.01 percent calculation error
    EXPECT_LT(new_state(0), radius + 0.1);  // x
    EXPECT_LT(new_state(1), radius + 0.1);  // y
    EXPECT_NEAR(new_state(0) * new_state(0) + new_state(1) * new_state(1), radius * radius,
                0.0001 * radius * radius);  // Radius
    EXPECT_LT(new_state(2), 2 * M_PI);      // theta
  }
}

/* ----------------------- IMU VELOCITY MODEL -------------------------*/

TEST(IMU_VELOCITY_MODEL, STANDING_STILL_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  ImuVelocityModel motion_model = ImuVelocityModel(r_matrix);
  MotionUpdate prediction_data = {0, 0, 0, 0, 0, rclcpp::Clock().now()};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = g_matrix * Eigen::MatrixXf::Zero(10, 10) * g_matrix.transpose();
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
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  ImuVelocityModel motion_model = ImuVelocityModel(r_matrix);
  MotionUpdate prediction_data = {0, 1, 0, 0, 0, rclcpp::Clock().now()};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = g_matrix * Eigen::MatrixXf::Ones(10, 10) * g_matrix.transpose();
  EXPECT_DOUBLE_EQ(new_state(0), 0.5);  // x
  EXPECT_DOUBLE_EQ(new_state(1), 0.0);  // y
  EXPECT_DOUBLE_EQ(new_state(2), 0.0);  // theta
}

TEST(IMU_VELOCITY_MODEL, LINEAR_VELOCITY_CURVE_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  ImuVelocityModel motion_model = ImuVelocityModel(r_matrix);
  MotionUpdate prediction_data = {0, 5, 10, M_PI / 16, 0, rclcpp::Clock().now()};
  Eigen::VectorXf new_state =
      motion_model.predict_expected_state(Eigen::VectorXf::Zero(10), prediction_data, 0.1);
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance =
      g_matrix * Eigen::MatrixXf::Ones(10, 10) *
      g_matrix.transpose();                     // Covariance with ones to check if it is modified
  EXPECT_NEAR(new_state(0), 0.025, 0.0001);     // x
  EXPECT_NEAR(new_state(1), 0.05, 0.001);       // y
  EXPECT_NEAR(new_state(2), M_PI / 160, 0.1);   // theta
  EXPECT_NEAR(new_state(3), 0.5, 0.001);        // Vx
  EXPECT_NEAR(new_state(4), 1, 0.001);          // Vy
  EXPECT_NEAR(new_state(5), M_PI / 16, 0.001);  // rot vel
}

TEST(IMU_VELOCITY_MODEL, COMPLEX_MOVEMENT_TEST) {
  Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(6, 6);
  ImuVelocityModel motion_model = ImuVelocityModel(r_matrix);
  MotionUpdate prediction_data = {0, 0, 0, 0, 0, rclcpp::Clock().now()};
  Eigen::VectorXf temp_state;
  Eigen::VectorXf new_state = Eigen::VectorXf::Zero(10);
  double radius = 12.7324;
  new_state(0) = 0;
  new_state(1) = static_cast<float>(-radius);  // Radius of the circle
  Eigen::MatrixXf g_matrix =
      motion_model.get_motion_to_state_matrix(Eigen::VectorXf::Zero(10), prediction_data, 1.0);
  Eigen::MatrixXf new_covariance = Eigen::MatrixXf::Ones(10, 10);

  for (int i = 0; i < 1000; i++) {
    prediction_data.rotational_velocity = M_PI / 4;
    temp_state = new_state;
    new_state = motion_model.predict_expected_state(new_state, prediction_data, 0.1);
    g_matrix = motion_model.get_motion_to_state_matrix(new_state, prediction_data, 0.1);
    new_covariance = g_matrix * new_covariance * g_matrix.transpose();

    EXPECT_LT(new_state(0), radius + 0.1);  // x
    EXPECT_LT(new_state(1), radius + 0.1);  // y
    EXPECT_NEAR(new_state(0) * new_state(0) + new_state(1) * new_state(1), radius * radius,
                0.0001 * radius * radius);  // Radius
    EXPECT_LT(new_state(2), 2 * M_PI);      // theta
  }
}
