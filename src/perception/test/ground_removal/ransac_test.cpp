#include "ground_removal/ransac.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

// Non-owning deleter: does nothing.
template <typename T>
struct non_owning_deleter {
  void operator()(T*) const {}
};

#include <utils/plane.hpp>

/**
 * @brief Test class for setting up data and testing RANSAC algorithm.
 *
 */
class RANSACTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create point clouds on the stack.
    cloud.points.clear();
    cloud.points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 0.5});
    cloud.points.push_back(pcl::PointXYZI{0.0, 1.0, 0.0, 1.0});
    cloud.points.push_back(pcl::PointXYZI{0.0, 0.0, 1.0, 1.5});
    cloud.points.push_back(pcl::PointXYZI{0.0060, 0.0060, 0.0060, 2.0});
    cloud.points.push_back(pcl::PointXYZI{10, 10, 10, 2.5});

    empty_cloud.points.clear();

    cloud_3_points.points.clear();
    cloud_3_points.points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 0.5});
    cloud_3_points.points.push_back(pcl::PointXYZI{0.0, 1.0, 0.0, 1.0});
    cloud_3_points.points.push_back(pcl::PointXYZI{0.0, 0.0, 1.0, 1.5});
  }

  // Stack-allocated point clouds.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  pcl::PointCloud<pcl::PointXYZI> cloud_3_points;
};

/**
 * @brief Test Scenario: All points fit in the model (Epsilon threshold very high).
 *
 */
TEST_F(RANSACTest, TestBigEpsilon) {
  RANSAC ground_removal(10000, 1);  // Create on the stack.
  Plane plane;

  // Wrap stack cloud in a non-owning shared pointer.
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Only points that fit into the plane are considered as part of the plane (No
 * points close enough to the plane).
 *
 */
TEST_F(RANSACTest, TestCommonScenario) {
  RANSAC ground_removal(0.05, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

/**
 * @brief Test Scenario: The points in the plane and a close enough point is removed - 1 point left.
 *
 */
TEST_F(RANSACTest, TestCommonScenario2) {
  RANSAC ground_removal(0.5, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 1);
}

/**
 * @brief Test Scenario: The epsilon threshold is set to 0 - No points are removed.
 *
 */
TEST_F(RANSACTest, TestThresholdZero) {
  RANSAC ground_removal(0, 10);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 5);
}

/**
 * @brief Test Scenario: Number of repetitions is set to 0 - Expected a point cloud with 0 points.
 *
 */
TEST_F(RANSACTest, TestZeroRepetitions) {
  RANSAC ground_removal(100, 0);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Really small threshold. Only the points of the plane are considered as part
 * of the ground.
 *
 */
TEST_F(RANSACTest, TestSmallEpsilon) {
  RANSAC ground_removal(0.00000000000000001, 10);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

/**
 * @brief Test Scenario: Really great threshold - All points are considered as part of the plane.
 *
 */
TEST_F(RANSACTest, TestBigEpsilon2) {
  RANSAC ground_removal(1000000, 1);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: 0 points are expected after the ground
 * removal.
 *
 */
TEST_F(RANSACTest, TestCommonScenario3Points) {
  RANSAC ground_removal(100, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3_ptr(
      &cloud_3_points, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud3_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: The epsilon threshold is set to 0 - No
 * points are removed.
 *
 */
TEST_F(RANSACTest, Test3PointsThresholdZero) {
  RANSAC ground_removal(0, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3_ptr(
      &cloud_3_points, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(cloud3_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 3);
}

/**
 * @brief Test Scenario: Point cloud with 0 points - Must return a point cloud with 0 points also.
 *
 */
TEST_F(RANSACTest, TestEmptyPointCloud) {
  RANSAC ground_removal(100, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &empty_cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(empty_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with 0 points. Epsilon and repetitions set to 0 - Expected 0
 * points.
 *
 */
TEST_F(RANSACTest, TestEmptyPointCloud2) {
  RANSAC ground_removal(0, 0);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &empty_cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal.ground_removal(empty_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}
