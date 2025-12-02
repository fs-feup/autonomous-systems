#include "slam_solver/graph_slam_solver/graph_slam_solver.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

class MockDataAssociationModel : public DataAssociationModel {
public:
  MOCK_METHOD(Eigen::VectorXi, associate,
              (const Eigen::VectorXd& state, const Eigen::VectorXd& observations,
               const Eigen::MatrixXd& covariance, const Eigen::VectorXd& observation_confidences),
              (const, override));
};

class MockLandmarkFilter : public LandmarkFilter {
public:
  MOCK_METHOD(Eigen::VectorXd, filter,
              (const Eigen::VectorXd& observations, const Eigen::VectorXd& observation_confidences,
               Eigen::VectorXi& associations),
              (override));
  MOCK_METHOD(void, delete_landmarks, (const Eigen::VectorXd& some_landmarks), (override));
};

class MockV2PModel : public V2PMotionModel {
public:
  MOCK_METHOD(Eigen::Vector3d, get_pose_difference,
              (const Eigen::Vector3d& previous_pose, const Eigen::VectorXd& motion_data,
               double delta_t),
              (override));

  MOCK_METHOD(Eigen::Matrix3d, get_jacobian_pose,
              (const Eigen::Vector3d& previous_pose, const Eigen::VectorXd& motion_data,
               const double delta_t),
              (override));

  MOCK_METHOD(Eigen::MatrixXd, get_jacobian_motion_data,
              (const Eigen::Vector3d& previous_pose, const Eigen::VectorXd& motion_data,
               const double delta_t),
              (override));
};

class MockLoopClosure : public LoopClosure {
public:
  MOCK_METHOD(LoopClosure::Result, detect,
              (const Eigen::Vector3d& current_pose, const Eigen::VectorXd& map_cones,
               const Eigen::VectorXi& associations, const Eigen::VectorXd& observations),
              (const, override));
};

/**
 * @brief Whitebox integration test for the GraphSLAMSolver
 */
class GraphSlamSolverTest : public testing::Test {
public:
  GraphSlamSolverTest() : params() {
    mock_motion_model_ptr = std::make_shared<MockV2PModel>();
    mock_data_association_ptr = std::make_shared<MockDataAssociationModel>();
    mock_landmark_filter_ptr = std::make_shared<MockLandmarkFilter>();
    mock_loop_closure_ptr = std::make_shared<MockLoopClosure>();
    motion_model_ptr = mock_motion_model_ptr;
    landmark_filter_ptr = mock_landmark_filter_ptr;
    data_association_ptr = mock_data_association_ptr;
    loop_closure_ptr = mock_loop_closure_ptr;
    solver = std::make_shared<GraphSLAMSolver>(params, data_association_ptr, motion_model_ptr,
                                               landmark_filter_ptr, nullptr, loop_closure_ptr);
    params.slam_optimization_period_ = 0.0;
    this->solver->set_mission(common_lib::competition_logic::Mission::AUTOCROSS);
  }

  SLAMParameters params;
  std::shared_ptr<MockV2PModel> mock_motion_model_ptr;
  std::shared_ptr<MockDataAssociationModel> mock_data_association_ptr;
  std::shared_ptr<MockLandmarkFilter> mock_landmark_filter_ptr;
  std::shared_ptr<MockLoopClosure> mock_loop_closure_ptr;
  std::shared_ptr<V2PMotionModel> motion_model_ptr;
  std::shared_ptr<LandmarkFilter> landmark_filter_ptr;
  std::shared_ptr<DataAssociationModel> data_association_ptr;
  std::shared_ptr<LoopClosure> loop_closure_ptr;
  std::shared_ptr<GraphSLAMSolver> solver;
};

/**
 * @brief Test the GraphSLAMSolver in one iteration of inputs
 */
TEST_F(GraphSlamSolverTest, MotionAndObservation) {
  // Arrange
  Eigen::VectorXi associations_first = Eigen::VectorXi::Ones(4) * -1;
  Eigen::VectorXi associations_second = Eigen::VectorXi::Ones(4) * -1;
  EXPECT_CALL(*mock_motion_model_ptr, get_pose_difference)
      .Times(4)
      .WillRepeatedly(testing::Return(Eigen::Vector3d(1.1, 0.0, 0.0)));
EXPECT_CALL(*mock_motion_model_ptr, get_jacobian_motion_data)
    .Times(4)
    .WillRepeatedly(testing::Return(Eigen::Matrix3d::Identity() * 0.1));

  EXPECT_CALL(*mock_data_association_ptr, associate)
      .Times(2)
      .WillOnce(testing::Return(associations_first))
      .WillOnce(testing::Return(associations_second));

  Eigen::VectorXd eigen_cones_start(8);
  eigen_cones_start << 3.0, 1.0, 3.0, -1.0, 6.0, 1.0, 6.0, -1.0;
  Eigen::VectorXd eigen_cones_end(8);
  eigen_cones_end << -1.0, 1.0, -1.0, -1.0, 2.0, 1.0, 2.0, -1.0;

  EXPECT_CALL(*mock_landmark_filter_ptr, filter)
      .Times(2)
      .WillOnce(testing::Return(eigen_cones_start))
      .WillOnce(testing::Return(eigen_cones_end));
  EXPECT_CALL(*mock_landmark_filter_ptr, delete_landmarks).Times(2);

  EXPECT_CALL(*mock_loop_closure_ptr, detect)
      .Times(2)
      .WillRepeatedly(testing::Return(LoopClosure::Result{false, 0.0}));

  common_lib::structures::Velocities velocities;
  velocities.timestamp_ = solver->_last_pose_update_ + rclcpp::Duration(1, 0);
  velocities.velocity_x = 1.1;
  velocities.velocity_y = 0.0;
  velocities.rotational_velocity = 0.0;

  std::vector<common_lib::structures::Cone> cones_start, cones_end;
  cones_start.push_back(common_lib::structures::Cone(3.0, 1.0, "blue", 1.0, rclcpp::Clock().now()));
  cones_start.push_back(
      common_lib::structures::Cone(3.0, -1.0, "yellow", 1.0, rclcpp::Clock().now()));
  cones_start.push_back(common_lib::structures::Cone(6.0, 1.0, "blue", 1.0, rclcpp::Clock().now()));
  cones_start.push_back(
      common_lib::structures::Cone(6.0, -1.0, "yellow", 1.0, rclcpp::Clock().now()));
      
  cones_end.push_back(common_lib::structures::Cone(-1.0, 1.0, "blue", 1.0, rclcpp::Clock().now()));
  cones_end.push_back(
      common_lib::structures::Cone(-1.0, -1.0, "yellow", 1.0, rclcpp::Clock().now()));
  cones_end.push_back(common_lib::structures::Cone(2.0, 1.0, "blue", 1.0, rclcpp::Clock().now()));
  cones_end.push_back(
      common_lib::structures::Cone(2.0, -1.0, "yellow", 1.0, rclcpp::Clock().now()));

  // Act
  solver->add_observations(cones_start, rclcpp::Clock().now());
  solver->add_velocities(velocities);
  velocities.timestamp_ += rclcpp::Duration(1, 0);
  solver->add_velocities(velocities);
  velocities.timestamp_ += rclcpp::Duration(1, 0);
  solver->add_velocities(velocities);
  velocities.timestamp_ += rclcpp::Duration(1, 0);
  solver->add_velocities(velocities);
  velocities.timestamp_ += rclcpp::Duration(1, 0);
  solver->add_velocities(velocities);
  const common_lib::structures::Pose pose_before_observations = solver->get_pose_estimate();
  const std::vector<common_lib::structures::Cone> map_before_observations =
      solver->get_map_estimate();

  solver->add_observations(cones_end, rclcpp::Clock().now());
  const common_lib::structures::Pose pose_after_observations = solver->get_pose_estimate();
  const std::vector<common_lib::structures::Cone> map_after_observations =
      solver->get_map_estimate();

  // Assert
  EXPECT_NEAR(pose_before_observations.position.x, 4.4, 0.5);
  EXPECT_NEAR(pose_after_observations.position.x, 4.4, 0.2);
  EXPECT_EQ(map_before_observations.size(), 4);
  EXPECT_EQ(map_after_observations.size(), 8);
  EXPECT_NEAR(map_before_observations[0].position.x, 3.0, 0.2);
  EXPECT_NEAR(map_before_observations[1].position.x, 3.0, 0.2);
  EXPECT_NEAR(map_before_observations[2].position.x, 6.0, 0.2);
  EXPECT_NEAR(map_before_observations[3].position.x, 6.0, 0.2);
  EXPECT_NEAR(map_after_observations[0].position.x, 3.0, 0.2);
  EXPECT_NEAR(map_after_observations[1].position.x, 3.0, 0.2);
  EXPECT_NEAR(map_after_observations[2].position.x, 6.0, 0.2);
  EXPECT_NEAR(map_after_observations[3].position.x, 6.0, 0.2);
  EXPECT_NEAR(map_before_observations[0].position.y, 1.0, 0.2);
  EXPECT_NEAR(map_before_observations[1].position.y, -1.0, 0.2);
  EXPECT_NEAR(map_before_observations[2].position.y, 1.0, 0.2);
  EXPECT_NEAR(map_before_observations[3].position.y, -1.0, 0.2);
  EXPECT_NEAR(map_after_observations[0].position.y, 1.0, 0.2);
  EXPECT_NEAR(map_after_observations[1].position.y, -1.0, 0.2);
  EXPECT_NEAR(map_after_observations[2].position.y, 1.0, 0.2);
  EXPECT_NEAR(map_after_observations[3].position.y, -1.0, 0.2);
}
