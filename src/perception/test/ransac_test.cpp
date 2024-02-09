#include <gtest/gtest.h>
#include "ground_removal/ransac.hpp"

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
TEST_F(RANSACTest, Test1) {

    auto ground_removal = new RANSAC(10000, 1);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 0);
}

/**
 * @brief Test Scenario: Only points that fit into the plane are considered as part of the plane (No points close enough to the plane).
 * 
 */
TEST_F(RANSACTest, Test2) {
    auto ground_removal = new RANSAC(0.05, 100);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 2);
}

/**
 * @brief Test Scenario: The points in the plane and a close enough point is removed - 1 point left.
 * 
 */
TEST_F(RANSACTest, Test3) {
    auto ground_removal = new RANSAC(0.5, 100);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 1);
}


/**
 * @brief Test Scenario: The epsilon threshold is set to 0 - No points are removed.
 * 
 */
TEST_F(RANSACTest, Test4) {
    auto ground_removal = new RANSAC(0, 10);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 5);
}


/**
 * @brief Test Scenario: Number of repetitions is set to 0 - Expected a point cloud with 0 points.
 * 
 */
TEST_F(RANSACTest, Test5) {
    auto ground_removal = new RANSAC(100, 0);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 0);
}


/**
 * @brief Test Scenario: Really small threshold. Only the points of the plane are considered as part of the ground.
 * 
 */
TEST_F(RANSACTest, Test6) {
    auto ground_removal = new RANSAC(0.00000000000000001, 10);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 2);
}


/**
 * @brief Test Scenario: Really great threshold - All points are considered as part of the plane.
 * 
 */
TEST_F(RANSACTest, Test7) {
    auto ground_removal = new RANSAC(1000000, 1);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 0);
}


/**
 * @brief Test Scenario: Point cloud with only 3 points: 0 points are expected after the ground removal.
 * 
 */
TEST_F(RANSACTest, Test8) {
    auto ground_removal = new RANSAC(100, 100);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud_3_points, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 0);
}

/**
 * @brief Test Scenario: Point cloud with only 3 points: The epsilon threshold is set to 0 - No points are removed.
 * 
 */
TEST_F(RANSACTest, Test9) {
    auto ground_removal = new RANSAC(0, 100);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud_3_points, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 3);
}

/**
 * @brief Test Scenario: Point cloud with 0 points - Must return a point with 0 points also.
 * 
 */
TEST_F(RANSACTest, Test10) {
    auto ground_removal = new RANSAC(100, 100);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud_empty, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 0);
}


/**
 * @brief Test Scenario: Point cloud with 0 points. Epsilon and repetitions set to 0 - Expected 0 points.
 * 
 */
TEST_F(RANSACTest, Test11) {
    auto ground_removal = new RANSAC(0, 0);

    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    ground_removal->groundRemoval(pcl_cloud_empty, ground_removed_cloud);

    ASSERT_EQ(ground_removed_cloud.points.size(), 0);
}