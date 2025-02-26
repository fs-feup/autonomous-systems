/**
 * @file icp_test.cpp
 * @brief Unit tests for the ICP class.
 */

#include "icp/icp.hpp"

#include <gtest/gtest.h>

#include <memory>

/**
 * @class ICPSuite
 * @brief Test suite for the ICP class.
 */
class ICPSuite : public ::testing::Test {
public:
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud;

protected:
  /**
   * @brief Set up the test fixture.
   */
  void SetUp() override {
    source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    source_cloud->points.push_back(pcl::PointXYZI{3, -3, 0, 0});
    source_cloud->points.push_back(pcl::PointXYZI{2, -4, 0, 0});
    source_cloud->points.push_back(pcl::PointXYZI{4, -4, 0, 0});
    source_cloud->width = 1;
    source_cloud->height = 3;
  }
};

/**
 * @brief Test case to check if ICP initializes properly
 */
TEST_F(ICPSuite, Initialization) {
  ICP icp("../../src/perception/test/icp/icp_tests/basic_cloud.pcd", 0.1, 50, 1e-8, 1);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  ASSERT_NO_THROW(icp.executeICP(source_cloud, target_cloud));
}

/**
 * @brief Test case to check if ICP aligns source cloud properly
 */
TEST_F(ICPSuite, Alignment) {
  ICP icp("../../src/perception/test/icp/icp_tests/basic_cloud.pcd", 100.0, 300, 5, 5);

  const pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  const double fitness_score = icp.executeICP(source_cloud, aligned_cloud);

  // Ensure that the fitness score is not negative, indicating successful alignment
  ASSERT_GE(fitness_score, 0);
}

/**
 * @brief Test case to check if ICP fails to align properly
 */
TEST_F(ICPSuite, AlignmentFailed) {
  ICP icp("../../src/perception/test/icp/icp_tests/basic_cloud.pcd", 1.0, 50, 1e-8, 5);

  const pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  const double fitness_score = icp.executeICP(source_cloud, aligned_cloud);

  ASSERT_EQ(fitness_score, -1);
}
