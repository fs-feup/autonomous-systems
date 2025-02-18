#include "slam_solver/ekf_slam_solver.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(EKFSLAMSolverTest, test_observation_model_1) {
  Eigen::VectorXd state(3);
  state << 0, 0, 0;
  EKFSLAMSolver ekf_slam_solver(SLAMSolverParameters(), nullptr, nullptr);
  Eigen::VectorXd observations = ekf_slam_solver.observation_model(state, std::vector<int>());
  ASSERT_EQ(observations.size(), 0);
}

TEST(EKFSLAMSolverTest, test_observation_model_2) {
  Eigen::VectorXd state(5);
  state << 0, 0, 0, 1, 1;
  EKFSLAMSolver ekf_slam_solver(SLAMSolverParameters(), nullptr, nullptr);
  Eigen::VectorXd observations = ekf_slam_solver.observation_model(state, std::vector<int>({3}));
  ASSERT_EQ(observations.size(), 2);
  ASSERT_EQ(observations(0), 1);
  ASSERT_EQ(observations(1), 1);
}

TEST(EKFSLAMSolverTest, test_observation_model_3) {
  Eigen::VectorXd state(5);
  state << 0, 0, M_PI / 2.0, 1, 1;
  EKFSLAMSolver ekf_slam_solver(SLAMSolverParameters(), nullptr, nullptr);
  Eigen::VectorXd observations = ekf_slam_solver.observation_model(state, std::vector<int>({3}));
  ASSERT_EQ(observations.size(), 2);
  ASSERT_EQ(observations(0), 1);
  ASSERT_EQ(observations(1), -1);
}

TEST(EKFSLAMSolverTest, test_inverse_observation_model1) {
  Eigen::VectorXd state(3);
  state << 0, 0, 0;
  EKFSLAMSolver ekf_slam_solver(SLAMSolverParameters(), nullptr, nullptr);
  Eigen::VectorXd observations(2);
  observations << 0, 0;
  Eigen::VectorXd inverse_observations =
      ekf_slam_solver.inverse_observation_model(state, observations);
  ASSERT_EQ(inverse_observations.size(), 2);
  ASSERT_EQ(inverse_observations(0), 0);
  ASSERT_EQ(inverse_observations(1), 0);
}