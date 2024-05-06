#include <gtest/gtest.h>
#include "icp/icp.hpp"

class ICPSuite : public ::testing::Test {
  protected:
    void SetUp() override {
        source_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        source_cloud->points.push_back(pcl::PointXYZI{3, -3, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{2, -4, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{4, -4, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{-30, -30, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{-50, 50, 0, 0});
        source_cloud->points.push_back(pcl::PointXYZI{10, -81.5, 0, 0});
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud;
};

// Test case to check if ICP initializes properly
TEST_F(ICPSuite, Initialization) {

    auto icp = ICP("../../src/perception/test/icp_tests/basic_cloud.pcd", 0.1, 50, 1e-8, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ASSERT_NO_THROW(icp.executeICP(target_cloud, target_cloud));
}


// Test case to check if ICP aligns source cloud properly
TEST_F(ICPSuite, Alignment) {

    auto icp = ICP("../../src/perception/test/icp_tests/basic_cloud.pcd", 10.0, 50, 1e-8, 5);

    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    double fitness_score = icp.executeICP(source_cloud, aligned_cloud);

    // Ensure that the fitness score is not negative, indicating successful alignment
    ASSERT_GE(fitness_score, 0);
}

TEST_F(ICPSuite, AlignmentFailed) {

    auto icp = ICP("../../src/perception/test/icp_tests/basic_cloud.pcd", 1.0, 50, 1e-8, 5);

    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    double fitness_score = icp.executeICP(source_cloud, aligned_cloud);

    ASSERT_EQ(fitness_score, -1);
}
