/**
 * @file icp_test.cpp
 * @brief Unit tests for the ICP class.
 */

#include <gtest/gtest.h>
#include "icp/icp.hpp"

/**
 * @class ICPSuite
 * @brief Test suite for the ICP class.
 */
class ICPSuite : public ::testing::Test {
protected:
    /**
     * @brief Set up the test fixture.
     */
    void SetUp() override {
        source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        
        source_cloud->points.push_back(pcl::PointXYZI{3, -3, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{2, -4, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{4, -4, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{-30, -30, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{-50, 50, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{10, -81.5, 0, 0});
    }

public:
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> source_cloud;
};

/**
 * @brief Test case to check if ICP initializes properly
 * 
 */
TEST_F(ICPSuite, Initialization) {
    auto icp = ICP("../../src/perception/test/icp_tests/basic_cloud.pcd", 0.1, 50, 1e-8, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ASSERT_NO_THROW(icp.executeICP(source_cloud, target_cloud));
}

/**
 * @brief Test case to check if ICP aligns source cloud properly
 * 
 */
TEST_F(ICPSuite, Alignment) {
    auto icp = ICP("../../src/perception/test/icp_tests/basic_cloud.pcd", 10.0, 50, 1e-8, 5);

    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    double fitness_score = icp.executeICP(source_cloud, aligned_cloud);

    // Ensure that the fitness score is not negative, indicating successful alignment
    ASSERT_GE(fitness_score, 0);
}

/**
 * @brief Test case to check if ICP fails to align properly
 * 
 */
TEST_F(ICPSuite, AlignmentFailed) {
    auto icp = ICP("../../src/perception/test/icp_tests/basic_cloud.pcd", 1.0, 50, 1e-8, 5);

    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    double fitness_score = icp.executeICP(source_cloud, aligned_cloud);

    ASSERT_EQ(fitness_score, -1);
}
