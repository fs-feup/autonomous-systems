#include "gtest/gtest.h"
#include "kalman_filter/observation_models.hpp"


TEST(BASE_OBSERVATION_MODEL, INVERSE_OBSERVATION_MODEL_TEST) {
  ObservationModel observation_model;
  Eigen::VectorXf expected_state(3);
  expected_state << 0, 0, 0;
  ObservationData observation_data;
  observation_data.position.x = 1;
  observation_data.position.y = 1;
  Eigen::Vector2f result = observation_model.inverse_observation_model(expected_state, observation_data);
  Eigen::Vector2f expected_result(1, 1);
  ASSERT_EQ(result, expected_result);
}