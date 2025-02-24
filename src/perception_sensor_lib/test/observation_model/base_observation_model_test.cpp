#include "perception_sensor_lib/observation_model/base_observation_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(EKFSLAMSolverTest, test_observation_model_1) {
  Eigen::VectorXd state(3);
  state << 0, 0, 0;
  ObservationModel observation_model_;
  Eigen::VectorXd observations = observation_model_.observation_model(state, std::vector<int>());
  EXPECT_FLOAT_EQ(observations.size(), 0);
}

TEST(EKFSLAMSolverTest, test_observation_model_2) {
  Eigen::VectorXd state(5);
  state << 0, 0, 0, 1, 1;
  ObservationModel observation_model_;
  Eigen::VectorXd observations = observation_model_.observation_model(state, std::vector<int>({3}));
  EXPECT_FLOAT_EQ(observations.size(), 2);
  EXPECT_FLOAT_EQ(observations(0), 1);
  EXPECT_FLOAT_EQ(observations(1), 1);
}

TEST(EKFSLAMSolverTest, test_observation_model_3) {
  Eigen::VectorXd state(5);
  state << 0, 0, M_PI / 2.0, 1, 1;
  ObservationModel observation_model_;
  Eigen::VectorXd observations = observation_model_.observation_model(state, std::vector<int>({3}));
  EXPECT_FLOAT_EQ(observations.size(), 2);
  EXPECT_FLOAT_EQ(observations(0), 1);
  EXPECT_FLOAT_EQ(observations(1), -1);
}

TEST(EKFSLAMSolverTest, test_inverse_observation_model1) {
  Eigen::VectorXd state(3);
  state << 0, 0, 0;
  ObservationModel observation_model_;
  Eigen::VectorXd observations(2);
  observations << 0, 0;
  Eigen::VectorXd inverse_observations =
      observation_model_.inverse_observation_model(state, observations);
  EXPECT_FLOAT_EQ(inverse_observations.size(), 2);
  EXPECT_FLOAT_EQ(inverse_observations(0), 0);
  EXPECT_FLOAT_EQ(inverse_observations(1), 0);
}