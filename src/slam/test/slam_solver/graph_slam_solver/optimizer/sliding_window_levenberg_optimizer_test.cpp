#include "slam_solver/graph_slam_solver/optimizer/sliding_window_levenberg_optimizer.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <yaml-cpp/yaml.h>

#include <iostream>

/**
 * @brief Whitebox unit test for the SlidingWindowLevenbergOptimizer
 */
class SlidingWindowLevenbergOptimizerTest : public testing::Test {
public:
  SlidingWindowLevenbergOptimizerTest() {}

protected:
  gtsam::NonlinearFactorGraph factor_graph;
  gtsam::Values graph_values;
  SLAMParameters params;
  std::shared_ptr<SlidingWindowLevenbergOptimizer> optimizer;

  void SetUp() override {
    // Set up any necessary data or state before each test
    factor_graph = gtsam::NonlinearFactorGraph();
    graph_values = gtsam::Values();
    params = SLAMParameters();
  }

  void TearDown() override {
    // Clean up any resources or state after each test
    factor_graph.resize(0);
    graph_values.clear();
  }

  /**
   * @brief Test the SlidingWindowLevenbergOptimizer
   * - num_poses: number of poses to add to the graph
   * - num_landmarks: number of landmarks to add to the graph
   * - window_size: size of the sliding window
   * - This test will add a specified number of poses and landmarks to the graph,
   *   add between factors and bearing range factors, and then optimize the graph using
   *  the SlidingWindowLevenbergOptimizer.
   * - It will check that the optimized values have changed, using wild values and factors.
   * - NOTE: if the test is issuing an error, it is possible that, despite weird values,
   * the optimized value is the same as the input value.
   */
  void test_sliding_window(unsigned int num_poses, unsigned int num_landmarks,
                           unsigned int window_size) {
    // ARRANGE
    params.sliding_window_size_ = window_size;  // Set sliding window size to 5
    optimizer = std::make_shared<SlidingWindowLevenbergOptimizer>(params);

    // Add some pose values to the graph
    for (unsigned int i = 1; i <= num_poses; i++) {
      gtsam::Key key = gtsam::Symbol('x', i);
      graph_values.insert(key, gtsam::Pose2(i * 5, i * 5, 0));
    }

    // Add some landmark values to the graph
    for (unsigned int j = 1; j <= num_landmarks; j++) {
      gtsam::Key key = gtsam::Symbol('l', j);
      graph_values.insert(key, gtsam::Point2(j * 5 + 5, 0));
    }

    // Add between factors
    for (unsigned int i = 1; i <= num_poses; i++) {
      gtsam::Key key = gtsam::Symbol('x', i);
      gtsam::Key key2 = gtsam::Symbol('x', i + 1);
      factor_graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
          key, key2, gtsam::Pose2(3, 1, 0),
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.2))));
    }

    // Add bearing range factors
    for (unsigned int landmark_index = 1; landmark_index <= num_landmarks; landmark_index++) {
      gtsam::Key key = gtsam::Symbol('l', landmark_index);
      for (unsigned int pose_index = 1; pose_index <= num_poses; ++pose_index) {
        factor_graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
            gtsam::Symbol('x', pose_index), key, gtsam::Rot2(0), 1,
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1))));
      }
    }

    // ACT
    gtsam::Values optimized_values;
    gtsam::Values old_values = graph_values;
    EXPECT_NO_THROW(optimized_values =
                        optimizer->optimize(factor_graph, graph_values, num_poses, num_landmarks));

    // ASSERT
    EXPECT_EQ(optimized_values.size(), num_poses + num_landmarks);
    for (unsigned int i = 1; i <= num_poses; i++) {
      EXPECT_TRUE(optimized_values.exists(gtsam::Symbol('x', i)));
    }
    for (unsigned int j = 1; j <= num_landmarks; j++) {
      EXPECT_TRUE(optimized_values.exists(gtsam::Symbol('l', j)));
    }

    // Assert that only the last 5 poses are optimized
    for (int i = 1; i <= static_cast<int>(num_poses - window_size); i++) {
      gtsam::Pose2 optimized_pose = optimized_values.at<gtsam::Pose2>(gtsam::Symbol('x', i));
      gtsam::Pose2 old_pose = old_values.at<gtsam::Pose2>(gtsam::Symbol('x', i));
      EXPECT_TRUE(optimized_pose.equals(old_pose));
    }

    // Assert that the optimized values are close to the expected values, but not equal
    for (unsigned int i = num_poses - window_size + 1; i <= num_poses; i++) {
      gtsam::Pose2 optimized_pose = optimized_values.at<gtsam::Pose2>(gtsam::Symbol('x', i));
      gtsam::Pose2 old_pose = old_values.at<gtsam::Pose2>(gtsam::Symbol('x', i));
      EXPECT_FALSE(optimized_pose.equals(old_pose) && num_landmarks != 0);
    }

    for (unsigned int landmark_index = 1; landmark_index <= num_landmarks; landmark_index++) {
      gtsam::Point2 optimized_landmark =
          optimized_values.at<gtsam::Point2>(gtsam::Symbol('l', landmark_index));
      gtsam::Point2 old_landmark = old_values.at<gtsam::Point2>(gtsam::Symbol('l', landmark_index));
      EXPECT_FALSE(optimized_landmark.x() == old_landmark.x() && num_poses != 0);
      EXPECT_FALSE(optimized_landmark.y() == old_landmark.y() && num_poses != 0);
    }
  }
};

/**
 * @brief Test the SlidingWindowLevenbergOptimizer
 * - less poses than sliding window
 * - just poses, no landmarks
 */
TEST_F(SlidingWindowLevenbergOptimizerTest, LessPosesThanSlidingWindowJustPoses) {
  test_sliding_window(3, 0, 5);
}

/**
 * @brief Test the SlidingWindowLevenbergOptimizer
 * - no poses
 * - no landmarks
 */
TEST_F(SlidingWindowLevenbergOptimizerTest, EmptyValuesAndGraph) { test_sliding_window(0, 0, 5); }

/**
 * @brief Test the SlidingWindowLevenbergOptimizer
 * - no poses
 * - just landmarks
 */
TEST_F(SlidingWindowLevenbergOptimizerTest, JustLandmarks) { test_sliding_window(0, 3, 5); }

/**
 * @brief Test the SlidingWindowLevenbergOptimizer
 * - less poses than sliding window
 * - poses and landmarks
 */
TEST_F(SlidingWindowLevenbergOptimizerTest, LessPosesThanSlidingWindow) {
  test_sliding_window(3, 3, 5);
}

/**
 * @brief Test the SlidingWindowLevenbergOptimizer
 * - same poses than sliding window
 * - poses and landmarks
 */
TEST_F(SlidingWindowLevenbergOptimizerTest, SamePosesThanSlidingWindow) {
  test_sliding_window(5, 3, 5);
}

/**
 * @brief Test the SlidingWindowLevenbergOptimizer
 * - more poses than sliding window
 * - poses and landmarks
 */
TEST_F(SlidingWindowLevenbergOptimizerTest, MorePosesThanSlidingWindow) {
  test_sliding_window(10, 3, 5);
}
