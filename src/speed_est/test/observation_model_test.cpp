#include <eigen3/Eigen/Dense>

#include "gtest/gtest.h"
#include "kalman_filter/observation_models.hpp"

/**
 * @brief Test the observation model in a scenario
 * where the vehicle is always aligned with the x axis
 *
 */
TEST(BASE_OBSERVATION_MODEL, ALIGNED_RANDOM_TEST) {
  Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  ObservationModel observation_model(Q);
  ObservationData observation_data;
  for (unsigned int i = 0; i < 1000; i++) {
    Eigen::VectorXf expected_state(5);
    double vehicle_x, vehicle_y, landmark_x, landmark_y;
    vehicle_x = rand() % 1000 - 500;
    vehicle_y = rand() % 1000 - 500;
    landmark_x = rand() % 1000 - 500;
    landmark_y = rand() % 1000 - 500;
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

// /**
//  * @brief Test the observation model in a scenario
//  * where the vehicle can have different orientations
//  *
//  */
// TEST(BASE_OBSERVATION_MODEL, CURVED_RANDOM_TEST) {
//   Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
//   ObservationModel observation_model(Q);
//   ObservationData observation_data;
//   for (unsigned int i = 0; i < 1000; i++) {
//     Eigen::VectorXf expected_state(5);
//     double vehicle_x, vehicle_y, landmark_x, landmark_y, orientation;
//     vehicle_x = rand() % 1000 - 500;
//     vehicle_y = rand() % 1000 - 500;
//     orientation = rand() % 2 * M_PI;
//     landmark_x = rand() % 1000 - 500;
//     landmark_y = rand() % 1000 - 500;

//     expected_state << vehicle_x, vehicle_y, orientation, landmark_x,
//     landmark_y; observation_data.position.x = landmark_x - vehicle_x;
//     observation_data.position.y = landmark_y - vehicle_y;
//     Eigen::Vector2f observed_landmark_position =
//         observation_model.inverse_observation_model(expected_state,
//         observation_data);
//     Eigen::Vector2f predicted_observation =
//     observation_model.observation_model(expected_state, 3);
//     EXPECT_EQ(observed_landmark_position(0), landmark_x);
//     EXPECT_EQ(observed_landmark_position(1), landmark_y);
//     EXPECT_EQ(predicted_observation(0), observation_data.position.x);
//     EXPECT_EQ(predicted_observation(1), observation_data.position.y);
//   }
// }

/**
 * @brief Test different cases for known values individually
 *
 */
TEST(BASE_OBSERVATION_MODEL, INDIVIDUAL_CASE_TEST) {
  // Test case 1 - 45 degrees to the left
  Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  ObservationModel observation_model(Q);
  ObservationData observation_data;
  Eigen::VectorXf expected_state(5);
  double vehicle_x, vehicle_y, landmark_x, landmark_y, orientation;
  vehicle_x = 5;
  vehicle_y = -2;
  orientation = -M_PI / 4;
  landmark_x = 5;
  landmark_y = -2 - sqrt(2);
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

  // Test case 2 - straight forward, 360 degrees
  vehicle_x = 5;
  vehicle_y = -2;
  orientation = 2 * M_PI;
  landmark_x = 6;
  landmark_y = -3;
  observation_data.position.x = 1;
  observation_data.position.y = -1;
  expected_state << vehicle_x, vehicle_y, orientation, landmark_x, landmark_y;
  observed_landmark_position =
      observation_model.inverse_observation_model(expected_state, observation_data);
  predicted_observation = observation_model.observation_model(expected_state, 3);
  EXPECT_NEAR(observed_landmark_position(0), landmark_x, 0.00001);
  EXPECT_NEAR(observed_landmark_position(1), landmark_y, 0.00001);
  EXPECT_NEAR(predicted_observation(0), observation_data.position.x, 0.00001);
  EXPECT_NEAR(predicted_observation(1), observation_data.position.y, 0.00001);

  // Test case 3 - 90 degrees to the right
  vehicle_x = 5;
  vehicle_y = -2;
  orientation = M_PI / 2;
  landmark_x = 6;
  landmark_y = -1;
  observation_data.position.x = 1;
  observation_data.position.y = -1;
  expected_state << vehicle_x, vehicle_y, orientation, landmark_x, landmark_y;
  observed_landmark_position =
      observation_model.inverse_observation_model(expected_state, observation_data);
  predicted_observation = observation_model.observation_model(expected_state, 3);
  EXPECT_NEAR(observed_landmark_position(0), landmark_x, 0.00001);
  EXPECT_NEAR(observed_landmark_position(1), landmark_y, 0.00001);
  EXPECT_NEAR(predicted_observation(0), observation_data.position.x, 0.00001);
  EXPECT_NEAR(predicted_observation(1), observation_data.position.y, 0.00001);
}
