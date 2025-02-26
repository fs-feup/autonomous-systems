#include "clustering/dbscan.hpp"

#include <gtest/gtest.h>

#include <memory>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

/**
 * @brief Test fixture for the DBSCAN class.
 *
 * The DBSCANTest class provides a test fixture for testing the DBSCAN clustering algorithm.
 */
class DBSCANTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create point cloud on the stack
    point_cloud.clear();

    point_cloud.push_back(pcl::PointXYZI(1.0, 2.0, 0.0, 0.1));
    point_cloud.push_back(pcl::PointXYZI(4.0, 5.0, 0.0, 0.2));
    point_cloud.push_back(pcl::PointXYZI(7.0, 8.0, 0.0, 0.3));
    point_cloud.push_back(pcl::PointXYZI(4.0, 3.5, 0.0, 0.4));
    point_cloud.push_back(pcl::PointXYZI(0, 7.0, 0.0, 0.5));
    point_cloud.push_back(pcl::PointXYZI(4.0, 4.0, 0.0, 0.6));
    point_cloud.push_back(pcl::PointXYZI(5.0, 4.5, 0.0, 0.7));
    point_cloud.push_back(pcl::PointXYZI(40.0, 40.0, 0.0, 0.8));
  }
  // Declare a stack-allocated point cloud object.
  pcl::PointCloud<pcl::PointXYZI> point_cloud;
};

/**
 * @brief Minimum values for cluster size and distance threshold
 *
 */
TEST_F(DBSCANTest, TestZeroNieghboursZeroEpsilon) {
  const auto clustering = std::make_unique<DBSCAN>(0, 0);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 8);
}

/**
 * @brief Large distance threshold (Expecting two clusters).
 *
 */
TEST_F(DBSCANTest, TestLargeEpsilon) {
  const auto clustering = std::make_unique<DBSCAN>(0, 30);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 2);
}

/**
 * @brief Minimum cluster size requirement with one outlier (Expecting one large cluster).
 *
 */
TEST_F(DBSCANTest, TestMinClusterSize) {
  const auto clustering = std::make_unique<DBSCAN>(2, 30);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Large distance threshold (Expecting one large cluster)
 *
 */
TEST_F(DBSCANTest, TestLargeEpsilon2) {
  const auto clustering = std::make_unique<DBSCAN>(0, 50);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Smaller distance threshold (Expecting multiple clusters)
 *
 */
TEST_F(DBSCANTest, TestSmallThreshold) {
  const auto clustering = std::make_unique<DBSCAN>(0, 1.5);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 5);
}

/**
 * @brief Minimum cluster size requirement with small distance threshold
 *
 */
TEST_F(DBSCANTest, TestMinClusterSize2) {
  const auto clustering = std::make_unique<DBSCAN>(2, 1.5);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Small distance threshold (Expecting all points as separate clusters)
 *
 */
TEST_F(DBSCANTest, TestSmallThreshold3) {
  const auto clustering = std::make_unique<DBSCAN>(0, 0.4);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 8);
}

/**
 * @brief Distance for aggregate the two nearest points (Expecting one less cluster).
 *
 */
TEST_F(DBSCANTest, TestAggregate2Points) {
  const auto clustering = std::make_unique<DBSCAN>(0, 0.6);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 7);
}

/**
 * @brief Large minimum cluster size requirement (Expecting no clusters).
 *
 */
TEST_F(DBSCANTest, TestMorePointsThanThreshold) {
  const auto clustering = std::make_unique<DBSCAN>(10, 50);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 0);
}

/**
 * @brief Small minimum cluster size requirement with very small distance
 *
 */
TEST_F(DBSCANTest, TestMinClusterWithSmallThreshold) {
  const auto clustering = std::make_unique<DBSCAN>(1, 0.1);

  // Create a non-owning shared pointer from our stack object.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &point_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  std::vector<Cluster> clusters;
  clustering->clustering(cloud_ptr, &clusters);

  ASSERT_EQ(clusters.size(), 8);
}
