#include "ground_removal/ransac.hpp"

#include <gtest/gtest.h>
#include <utils/plane.hpp>

/**
 * @brief Test class for setting up data and testing RANSAC algorithm.
 *
 */
class RANSACTest : public ::testing::Test {
 protected:
  /**
   * @brief Set up the test environment before each test case.
   *
   */
  void SetUp() override {
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_cloud->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 0.5});
    pcl_cloud->points.push_back(pcl::PointXYZI{0.0, 1.0, 0.0, 1.0});
    pcl_cloud->points.push_back(pcl::PointXYZI{0.0, 0.0, 1.0, 1.5});
    pcl_cloud->points.push_back(pcl::PointXYZI{0.0060, 0.0060, 0.0060, 2.0});
    pcl_cloud->points.push_back(pcl::PointXYZI{10, 10, 10, 2.5});

    pcl_cloud_empty.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl_cloud_3_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_cloud_3_points->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 0.5});
    pcl_cloud_3_points->points.push_back(pcl::PointXYZI{0.0, 1.0, 0.0, 1.0});
    pcl_cloud_3_points->points.push_back(pcl::PointXYZI{0.0, 0.0, 1.0, 1.5});
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_empty;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_3_points;
};

/**
 * @brief Test Scenario: All points fit in the model (Epsilon threshold very high).
 *
 */
TEST_F(RANSACTest, TestBigEpsilon) {
  auto ground_removal = new RANSAC(10000, 1);

  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Only points that fit into the plane are considered as part of the plane (No
 * points close enough to the plane).
 *
 */
TEST_F(RANSACTest, TestCommonScenario) {
  auto ground_removal = new RANSAC(0.05, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

/**
 * @brief Test Scenario: The points in the plane and a close enough point is removed - 1 point left.
 *
 */
TEST_F(RANSACTest, TestCommonScenario2) {
  auto ground_removal = new RANSAC(0.5, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 1);
}

/**
 * @brief Test Scenario: The epsilon threshold is set to 0 - No points are removed.
 *
 */
TEST_F(RANSACTest, TestThresholdZero) {
  auto ground_removal = new RANSAC(0, 10);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 5);
}

/**
 * @brief Test Scenario: Number of repetitions is set to 0 - Expected a point cloud with 0 points.
 *
 */
TEST_F(RANSACTest, TestZeroRepetitions) {
  auto ground_removal = new RANSAC(100, 0);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Really small threshold. Only the points of the plane are considered as part
 * of the ground.
 *
 */
TEST_F(RANSACTest, TestSmallEpsilon) {
  auto ground_removal = new RANSAC(0.00000000000000001, 10);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

/**
 * @brief Test Scenario: Really great threshold - All points are considered as part of the plane.
 *
 */
TEST_F(RANSACTest, TestBigEpsilon2) {
  auto ground_removal = new RANSAC(1000000, 1);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: 0 points are expected after the ground
 * removal.
 *
 */
TEST_F(RANSACTest, TestCommonScenario3Points) {
  auto ground_removal = new RANSAC(100, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_3_points, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: The epsilon threshold is set to 0 - No
 * points are removed.
 *
 */
TEST_F(RANSACTest, Test3PointsThresholdZero) {
  auto ground_removal = new RANSAC(0, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_3_points, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 3);
}

/**
 * @brief Test Scenario: Point cloud with 0 points - Must return a point with 0 points also.
 *
 */
TEST_F(RANSACTest, TestEmptyPointCloud) {
  auto ground_removal = new RANSAC(100, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_empty, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with 0 points. Epsilon and repetitions set to 0 - Expected 0
 * points.
 *
 */
TEST_F(RANSACTest, TestEmptyPointCloud2) {
  auto ground_removal = new RANSAC(0, 0);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_empty, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}