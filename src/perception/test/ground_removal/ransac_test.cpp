#include "ground_removal/ransac.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <test_utils/pointcloud2_helper.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test class for setting up data and testing RANSAC algorithm.
 *
 */
class RANSACTest : public ::testing::Test {
protected:
  void SetUp() override {
    cloud_pts = {{1.0, 0.0, 0.0, 0.5, 0},
                 {0.0, 1.0, 0.0, 1.0, 0},
                 {0.0, 0.0, 1.0, 1.5, 0},
                 {0.0060, 0.0060, 0.0060, 2.0, 0},
                 {10, 10, 10, 2.5, 0}};
    empty_cloud_pts = {};
    cloud_3_pts = {{1.0, 0.0, 0.0, 0.5, 0}, {0.0, 1.0, 0.0, 1.0, 0}, {0.0, 0.0, 1.0, 1.5, 0}};
  }

  std::vector<std::array<float, 5>> cloud_pts;
  std::vector<std::array<float, 5>> empty_cloud_pts;
  std::vector<std::array<float, 5>> cloud_3_pts;

  GroundGrid default_grid{30.0, 0.1, 0.5, 10.0, 0.1, 3.14};
};

/**
 * @brief Test Scenario: All points fit in the model.
 *
 */
TEST_F(RANSACTest, TestBigEpsilon) {
  const RANSAC ground_removal(10'000, 1);
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}

/**
 * @brief Test Scenario: Only points that fit into the plane are considered as part of the plane.
 *
 */
TEST_F(RANSACTest, TestCommonScenario) {
  const RANSAC ground_removal(0.05, 100);
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 2);
}

/**
 * @brief Test Scenario: The points in the plane and a close enough point is removed - 1 point left.
 *
 */
TEST_F(RANSACTest, TestCommonScenario2) {
  const RANSAC ground_removal(0.5, 100);
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 1);
}

/**
 * @brief Test Scenario: The epsilon threshold is set to 0 - No points are removed.
 *
 */
TEST_F(RANSACTest, TestThresholdZero) {
  const RANSAC ground_removal(0, 10);
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 5);
}

/**
 * @brief Test Scenario: Number of repetitions is set to 0 - Expected a point cloud with 0 points.
 *
 */
TEST_F(RANSACTest, TestZeroRepetitions) {
  const RANSAC ground_removal(100, 0);
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}

/**
 * @brief Test Scenario: Really small threshold. Only the points of the plane are considered as part
 * of the ground.
 *
 */
TEST_F(RANSACTest, TestSmallEpsilon) {
  const RANSAC ground_removal(0.000'000'000'000'000'01, 40);
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 2);
}

/**
 * @brief Test Scenario: Really great threshold - All points are considered as part of the plane.
 *
 */
TEST_F(RANSACTest, TestBigEpsilon2) {
  const RANSAC ground_removal(1'000'000, 1);
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: 0 points are expected after the ground
 * removal.
 *
 */
TEST_F(RANSACTest, TestCommonScenario3Points) {
  const RANSAC ground_removal(100, 100);
  auto cloud3_ptr = test_utils::make_lidar_pointcloud2(cloud_3_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud3_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: The epsilon threshold is set to 0 - No
 * points are removed.
 *
 */
TEST_F(RANSACTest, Test3PointsThresholdZero) {
  const RANSAC ground_removal(0, 100);
  auto cloud3_ptr = test_utils::make_lidar_pointcloud2(cloud_3_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(cloud3_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 3);
}

/**
 * @brief Test Scenario: Point cloud with 0 points - Must return a point cloud with 0 points also.
 *
 */
TEST_F(RANSACTest, TestEmptyPointCloud) {
  const RANSAC ground_removal(100, 100);
  auto empty_ptr = test_utils::make_lidar_pointcloud2(empty_cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(empty_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}

/**
 * @brief Test Scenario: Point cloud with 0 points. Epsilon and repetitions set to 0 - Expected 0
 * points.
 *
 */
TEST_F(RANSACTest, TestEmptyPointCloud2) {
  const RANSAC ground_removal(0, 0);
  auto empty_ptr = test_utils::make_lidar_pointcloud2(empty_cloud_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_removal.ground_removal(empty_ptr, ground_removed_cloud_ptr, default_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}
