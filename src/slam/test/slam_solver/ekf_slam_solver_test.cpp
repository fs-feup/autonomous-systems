#include "slam_solver/ekf_slam_solver.hpp"

#include <gtest/gtest.h>

#include "motion_lib/v2p_models/map.hpp"
#include "perception_sensor_lib/data_association/map.hpp"

class EKFSLAMSolverTest : public ::testing::Test {
protected:
  void SetUp() override {
    SLAMParameters params;
    params.load_config();
    data_association = data_association_models_map.at(params.data_association_model_name_)(
        DataAssociationParameters(params.data_association_limit_distance_,
                                  params.data_association_gate_,
                                  params.new_landmark_confidence_gate_, params.observation_x_noise_,
                                  params.observation_y_noise_));
    motion_model = v2p_models_map.at(params.motion_model_name_)();

    ekf_slam_solver = std::make_shared<EKFSLAMSolver>(params, data_association, motion_model,
                                                      execution_time, node);
  }

  std::shared_ptr<DataAssociationModel> data_association;
  std::shared_ptr<V2PMotionModel> motion_model;
  std::shared_ptr<EKFSLAMSolver> ekf_slam_solver;
  std::shared_ptr<std::vector<double>> execution_time;
  std::weak_ptr<rclcpp::Node> node;
};

/**
 * @brief Test the state augmentation function of the EKF SLAM solver with trivial state
 *
 */
TEST_F(EKFSLAMSolverTest, stateAugmentation) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd new_landmarks_coordinates(10);
  Eigen::VectorXd new_landmarks_confidences(5);
  new_landmarks_coordinates << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  new_landmarks_confidences << 0.1, 0.2, 0.3, 0.4, 0.5;
  ekf_slam_solver->state_augmentation(state, covariance, new_landmarks_coordinates,
                                      new_landmarks_confidences);
  std::vector<double> expected_state = {0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  EXPECT_EQ(state.size(), expected_state.size());
  for (int i = 0; i < state.size(); i++) {
    EXPECT_NEAR(state(i), expected_state[i], 0.0001);
  }
}

/**
 * @brief Test the state augmentation function of the EKF SLAM solver with non-trivial state
 *
 */
TEST_F(EKFSLAMSolverTest, stateAugmentation2) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(3);
  state << 0, 0, M_PI / 2;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd new_landmarks_coordinates(10);
  Eigen::VectorXd new_landmarks_confidences(5);
  new_landmarks_coordinates << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  new_landmarks_confidences << 0.1, 0.2, 0.3, 0.4, 0.5;
  ekf_slam_solver->state_augmentation(state, covariance, new_landmarks_coordinates,
                                      new_landmarks_confidences);
  std::vector<double> expected_state = {0, 0, M_PI / 2, -2, 1, -4, 3, -6, 5, -8, 7, -10, 9};
  EXPECT_EQ(state.size(), expected_state.size());
  for (int i = 0; i < state.size(); i++) {
    EXPECT_NEAR(state(i), expected_state[i], 0.0001);
  }
}