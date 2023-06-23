#include "gtest/gtest.h"
#include "kalman_filter/observation_models.hpp"

TEST(BASE_OBSERVATION_MODEL, SIMPLE_RANDOM_TEST) {
  ObservationModel observation_model;
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
