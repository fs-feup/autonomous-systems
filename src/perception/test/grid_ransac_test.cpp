#include "ground_removal/grid_ransac.hpp"

#include <gtest/gtest.h>

#include <utils/plane.hpp>

/**
 * @brief Test class for setting up data and testing Grid RANSAC algorithm.
 *
 */
class GridRANSACTest : public ::testing::Test {
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

    pcl_cloud_two_grids.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_cloud_two_grids->points.push_back(pcl::PointXYZI{55.0, 55.0, 100.0, 0.5});
    pcl_cloud_two_grids->points.push_back(pcl::PointXYZI{60.0, 60.0, 100.0, 1.0});
    pcl_cloud_two_grids->points.push_back(pcl::PointXYZI{55.0, 55.0, 100.0, 1.5});
    pcl_cloud_two_grids->points.push_back(pcl::PointXYZI{-55.0, -55.0, 0.0, 0.5});
    pcl_cloud_two_grids->points.push_back(pcl::PointXYZI{-50.0, -50, 0.0, 1.0});
    pcl_cloud_two_grids->points.push_back(pcl::PointXYZI{-60.0, -60, 0.0, 1.5});

    pcl_cloud_four_grids.reset(new pcl::PointCloud<pcl::PointXYZI>);
    // first quadrant
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{55.0, 55.0, 100.0, 0.5});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{60.0, 60.0, 100.0, 1.0});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{55.0, 55.0, 3.0, 1.5});

    // second quadrant
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{55.0, -55.0, 0.0, 0.5});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{60.0, -60.0, 0.0, 1.0});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{55.0, -55.0, 0.0, 1.5});

    // third quadrant
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{-55.0, -55.0, -130.0, 0.5});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{-50.0, -50, -130.0, 1.0});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{-60.0, -60, -130.0, 1.5});

    // fourth quadrant
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{55.0, -55.0, 400.0, 0.5});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{50.0, -50, 400.0, 1.0});
    pcl_cloud_four_grids->points.push_back(pcl::PointXYZI{60.0, -60, 400.0, 1.5});

    pcl_two_radius.reset(new pcl::PointCloud<pcl::PointXYZI>); // r < 13
    pcl_two_radius->points.push_back(pcl::PointXYZI{7.0, 7.0, 400.0, 0.5});
    pcl_two_radius->points.push_back(pcl::PointXYZI{8, 4, 400.0, 1.0});
    pcl_two_radius->points.push_back(pcl::PointXYZI{9, 9, 400.0, 1.5});

    pcl_two_radius->points.push_back(pcl::PointXYZI{10, 10, -400.0, 0.5});
    pcl_two_radius->points.push_back(pcl::PointXYZI{11, 11, -400.0, 1.0});
    pcl_two_radius->points.push_back(pcl::PointXYZI{12, 13, -400.0, 1.5});
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_empty;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_3_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_two_grids;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_four_grids;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_two_radius;
};

/**
 * @brief Test Scenario: All points fit in the model (Epsilon threshold very high) (1 grid - Expected as equal as normal ransac).
 *
 */
TEST_F(GridRANSACTest, TestBigEpsilon) {
  auto ground_removal = new GridRANSAC(10000, 1, 1, 100);

  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Only points that fit into the plane are considered as part of the plane (No
 * points close enough to the plane) (1 grid - Expected as equal as normal ransac).
 *
 */
TEST_F(GridRANSACTest, TestCommonScenario) {
  auto ground_removal = new GridRANSAC(0.05, 100, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);
  

  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

/**
 * @brief Test Scenario: The points in the plane and a close enough point is removed - 1 point left.
 * (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestCommonScenario2) {
  auto ground_removal = new GridRANSAC(0.5, 100, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 1);
}

/**
 * @brief Test Scenario: The epsilon threshold is set to 0 - No points are removed.
 * (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestThresholdZero) {
  auto ground_removal = new GridRANSAC(0, 10, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 5);
}

/**
 * @brief Test Scenario: Number of repetitions is set to 0 - Expected a point cloud with 0 points.
 * (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestZeroRepetitions) {
  auto ground_removal = new GridRANSAC(100, 0, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Really small threshold. Only the points of the plane are considered as part
 * of the ground. (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestSmallEpsilon) {
  auto ground_removal = new GridRANSAC(0.00000000000000001, 10, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

/**
 * @brief Test Scenario: Really great threshold - All points are considered as part of the plane.
 * (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestBigEpsilon2) {
  auto ground_removal = new GridRANSAC(1000000, 1, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: 0 points are expected after the ground
 * removal. (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestCommonScenario3Points) {
  auto ground_removal = new GridRANSAC(100, 100, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_3_points, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: The epsilon threshold is set to 0 - No
 * points are removed. (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, Test3PointsThresholdZero) {
  auto ground_removal = new GridRANSAC(0, 100, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_3_points, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 3);
}

/**
 * @brief Test Scenario: Point cloud with 0 points - Must return a point with 0 points also.
 * (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestEmptyPointCloud) {
  auto ground_removal = new GridRANSAC(100, 100, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_empty, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with 0 points. Epsilon and repetitions set to 0 - Expected 0
 * points. (1 grid - Expected as equal as normal ransac)
 *
 */
TEST_F(GridRANSACTest, TestEmptyPointCloud2) {
  auto ground_removal = new GridRANSAC(0, 0, 1, 100);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_empty, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}


TEST_F(GridRANSACTest, TestTwoGrids) {
  auto ground_removal = new GridRANSAC(100, 1000, 2, 10000);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_two_grids, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}


TEST_F(GridRANSACTest, TestTwoGridsSinglePlane) {
  auto ground_removal = new GridRANSAC(0.000001, 1000, 1, 10000);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_two_grids, ground_removed_cloud, plane);

  ASSERT_NE(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGrids) {
  auto ground_removal = new GridRANSAC(100, 1000, 4, 10000);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_four_grids, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGridsTwoPlanes) {
  auto ground_removal = new GridRANSAC(0.000001, 1000, 2, 10000);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_four_grids, ground_removed_cloud, plane);

  ASSERT_NE(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGridsSinglePlane) {
  auto ground_removal = new GridRANSAC(0.000001, 1000, 1, 10000);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_cloud_four_grids, ground_removed_cloud, plane);

  ASSERT_NE(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoRadius) {
  auto ground_removal = new GridRANSAC(0.000001, 1000, 1, 13);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_two_radius, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoRadiusOnePlane) {
  auto ground_removal = new GridRANSAC(0.000001, 1000, 1, 10000);
  Plane plane;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->groundRemoval(pcl_two_radius, ground_removed_cloud, plane);

  ASSERT_NE(ground_removed_cloud->points.size(), 0);
}