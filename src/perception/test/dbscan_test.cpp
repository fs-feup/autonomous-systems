#include <gtest/gtest.h>
#include "clustering/dbscan.hpp"

/**
 * @brief Test fixture for the DBSCAN class.
 * 
 * The DBSCANTest class provides a test fixture for testing the DBSCAN clustering algorithm.
 */
class DBSCANTest : public ::testing::Test {
 protected:
    void SetUp() override {
        point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 0.0, 0.1));
        point_cloud->push_back(pcl::PointXYZI(4.0, 5.0, 0.0, 0.2));
        point_cloud->push_back(pcl::PointXYZI(7.0, 8.0, 0.0, 0.3));
        point_cloud->push_back(pcl::PointXYZI(4.0, 3.5, 0.0, 0.4));
        point_cloud->push_back(pcl::PointXYZI(0, 7.0, 0.0, 0.5));
        point_cloud->push_back(pcl::PointXYZI(4.0, 4.0, 0.0, 0.6));
        point_cloud->push_back(pcl::PointXYZI(5.0, 4.5, 0.0, 0.7));
        point_cloud->push_back(pcl::PointXYZI(40.0, 40.0, 0.0, 0.8));
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
};


/**
 * @brief Minimum values for cluster size and distance threshold
 * 
 */
TEST_F(DBSCANTest, TestZeroNieghboursZeroEpsilon) {
    auto clustering = new DBSCAN(0, 0);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 8);
}


/**
 * @brief Large distance threshold (Expecting two clusters).
 * 
 */
TEST_F(DBSCANTest, TestLargeEpsilon) {
    auto clustering = new DBSCAN(0, 30);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 2);
}

/**
 * @brief Minimum cluster size requirement with one outlier (Expecting one large cluster).
 * 
 */
TEST_F(DBSCANTest, TestMinClusterSize) {
    auto clustering = new DBSCAN(2, 30);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Large distance threshold (Expecting one large cluster)
 * 
 */
TEST_F(DBSCANTest, TestLargeEpsilon2) {
    auto clustering = new DBSCAN(0, 50);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Smaller distance threshold (Expecting multiple clusters)
 * 
 */
TEST_F(DBSCANTest, TestSmallThreshold) {
    auto clustering = new DBSCAN(0, 1.5);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 5);
}

/**
 * @brief Minimum cluster size requirement with small distance threshold 
 * 
 */
TEST_F(DBSCANTest, TestMinClusterSize2) {
    auto clustering = new DBSCAN(2, 1.5);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Small distance threshold (Expecting all points as separate clusters)
 * 
 */
TEST_F(DBSCANTest, TestSmallThreshold3) {
    auto clustering = new DBSCAN(0, 0.4);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 8);
}

/**
 * @brief Distance for aggregate the two nearest points (Expecting one less cluster).
 * 
 */
TEST_F(DBSCANTest, TestAggregate2Points) {
    auto clustering = new DBSCAN(0, 0.6);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 7);
}

/**
 * @brief Large minimum cluster size requirement (Expecting no clusters).
 * 
 */
TEST_F(DBSCANTest, TestMorePointsThanThreshold) {
    auto clustering = new DBSCAN(10, 50);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 0);
}

/**
 * @brief Small minimum cluster size requirement with very small distance 
 * 
 */
TEST_F(DBSCANTest, TestMinClusterWithSmallThreshold) {
    auto clustering = new DBSCAN(1, 0.1);

    std::vector<Cluster> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 8);
}

