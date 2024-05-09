#include <eigen3/Eigen/Dense>
#include <random>

#include "gtest/gtest.h"
#include "kalman_filter/observation_models.hpp"

/**
 * @brief Test the observation model in a scenario
 * where the vehicle is always aligned with the x axis
 *
 */
TEST(BASE_OBSERVATION_MODEL, ALIGNED_RANDOM_TEST) {
  Eigen::Matrix2f q_matrix = Eigen::Matrix2f::Zero();
  ObservationModel observation_model(q_matrix);
  ObservationData observation_data;
  std::random_device rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());
  std::uniform_int_distribution dis(-500, 500);
  for (unsigned int i = 0; i < 1000; i++) {
    Eigen::VectorXf expected_state(5);
    float vehicle_x = static_cast<float>(dis(gen));
    float vehicle_y = static_cast<float>(dis(gen));
    float landmark_x = static_cast<float>(dis(gen));
    float landmark_y = static_cast<float>(dis(gen));
    expected_state << vehicle_x, vehicle_y, 0, landmark_x, landmark_y;

    observation_data.position.x = landmark_x - vehicle_x;
    observation_data.position.y = landmark_y - vehicle_y;
    Eigen::Vector2f observed_landmark_position =
        observation_model.inverse_observation_model(expected_state, observation_data);
    Eigen::Vector2f predicted_observation = observation_model.observation_model(expected_state, 3);
    EXPECT_EQ(observed_landmark_position(0), landmark_x);
    EXPECT_EQ(observed_landmark_position(1), landmark_y);
    EXPECT_EQ(predicted_observation(0), observation_data.position.x);
    EXPECT_EQ(predicted_observation(1), observation_data.position.y);
  }
}

/**
 * @brief 45 degrees to the left
 *
 */
TEST(BASE_OBSERVATION_MODEL, INDIVIDUAL_CASE_TEST1) {
  Eigen::Matrix2f q_matrix = Eigen::Matrix2f::Zero();
  ObservationModel observation_model(q_matrix);
  ObservationData observation_data;
  Eigen::VectorXf expected_state(5);
  float vehicle_x = 5;
  float vehicle_y = -2;
  float orientation = static_cast<float>(-M_PI / 4);
  float landmark_x = 5;
  float landmark_y = static_cast<float>(-2 - sqrt(2));
  observation_data.position.x = 1;
  observation_data.position.y = -1;
  expected_state << vehicle_x, vehicle_y, orientation, landmark_x, landmark_y;
  Eigen::Vector2f observed_landmark_position =
      observation_model.inverse_observation_model(expected_state, observation_data);
  Eigen::Vector2f predicted_observation = observation_model.observation_model(expected_state, 3);
  EXPECT_NEAR(observed_landmark_position(0), landmark_x, 0.00001);
  EXPECT_NEAR(observed_landmark_position(1), landmark_y, 0.00001);
  EXPECT_NEAR(predicted_observation(0), observation_data.position.x, 0.00001);
  EXPECT_NEAR(predicted_observation(1), observation_data.position.y, 0.00001);
}

/**
 * @brief Straight forward 360 degrees
 *
 */
TEST(BASE_OBSERVATION_MODEL, INDIVIDUAL_CASE_TEST2) {
  Eigen::Matrix2f q_matrix = Eigen::Matrix2f::Zero();
  ObservationModel observation_model(q_matrix);
  ObservationData observation_data;
  Eigen::VectorXf expected_state(5);
  float vehicle_x = 5;
  float vehicle_y = -2;
  float orientation = static_cast<float>(2 * M_PI);
  float landmark_x = 6;
  float landmark_y = -3;
  observation_data.position.x = 1;
  observation_data.position.y = -1;
  expected_state << vehicle_x, vehicle_y, orientation, landmark_x, landmark_y;
  Eigen::Vector2f observed_landmark_position =
      observation_model.inverse_observation_model(expected_state, observation_data);
  Eigen::Vector2f predicted_observation = observation_model.observation_model(expected_state, 3);
  EXPECT_NEAR(observed_landmark_position(0), landmark_x, 0.00001);
  EXPECT_NEAR(observed_landmark_position(1), landmark_y, 0.00001);
  EXPECT_NEAR(predicted_observation(0), observation_data.position.x, 0.00001);
  EXPECT_NEAR(predicted_observation(1), observation_data.position.y, 0.00001);
}

/**
 * @brief 90 degrees to the right
 *
 */
TEST(BASE_OBSERVATION_MODEL, INDIVIDUAL_CASE_TEST3) {
  Eigen::Matrix2f q_matrix = Eigen::Matrix2f::Zero();
  ObservationModel observation_model(q_matrix);
  ObservationData observation_data;
  Eigen::VectorXf expected_state(5);
  float vehicle_x = 5;
  float vehicle_y = -2;
  float orientation = static_cast<float>(M_PI / 2);
  float landmark_x = 6;
  float landmark_y = -1;
  observation_data.position.x = 1;
  observation_data.position.y = -1;
  expected_state << vehicle_x, vehicle_y, orientation, landmark_x, landmark_y;
  Eigen::Vector2f observed_landmark_position =
      observation_model.inverse_observation_model(expected_state, observation_data);
  Eigen::Vector2f predicted_observation = observation_model.observation_model(expected_state, 3);
  EXPECT_NEAR(observed_landmark_position(0), landmark_x, 0.00001);
  EXPECT_NEAR(observed_landmark_position(1), landmark_y, 0.00001);
  EXPECT_NEAR(predicted_observation(0), observation_data.position.x, 0.00001);
  EXPECT_NEAR(predicted_observation(1), observation_data.position.y, 0.00001);
}
