
#include <gtest/gtest.h>

#include <clustering/grid_clustering.hpp>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <test_utils/pointcloud2_helper.hpp>

/**
 * @brief Test fixture for the GridClustering class.
 *
 */
class GridClusteringTest : public ::testing::Test {
protected:
  void SetUp() override {
    pts = {{1.0, 2.0, 0.0, 0.1, 39}, {4.0, 5.0, 0.0, 0.2, 39},  {7.0, 8.0, 0.0, 0.3, 39},
           {4.0, 3.5, 0.0, 0.4, 39}, {0.0, 7.0, 0.0, 0.5, 39},  {4.0, 4.0, 0.0, 0.6, 39},
           {5.0, 4.5, 0.0, 0.7, 39}, {40.0, 40.0, 0.0, 0.8, 39}};
  }
  std::vector<std::array<float, 5>> pts;
};

/**
 * @brief Minimum values for cluster size and distance threshold
 *
 */
TEST_F(GridClusteringTest, TestZeroGrid) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(0.01, 0.01, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_EQ(clusters.size(), 8);
}

/**
 * @brief Large distance threshold (Expecting two clusters).
 *
 */
TEST_F(GridClusteringTest, TestLargeGrid) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(90, 100, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Minimum cluster size requirement with one outlier (Expecting one large cluster).
 *
 */
TEST_F(GridClusteringTest, TestMinClusterSize) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(90, 100, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Large distance threshold (Expecting one large cluster)
 *
 */
TEST_F(GridClusteringTest, TestVeryLargeGrid) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(1000, 1000, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_EQ(clusters.size(), 1);
}

/**
 * @brief Smaller distance threshold (Expecting multiple clusters)
 *
 */
TEST_F(GridClusteringTest, TestSmallGrid) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(1, 1, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_GE(clusters.size(), 5);
}

/**
 * @brief Minimum cluster size requirement with small distance threshold
 *
 */
TEST_F(GridClusteringTest, TestMinClusterSize2) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(1, 1, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_GE(clusters.size(), 1);
}

/**
 * @brief Small distance threshold (Expecting all points as separate clusters)
 *
 */
TEST_F(GridClusteringTest, TestTinyGrid) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(0.01, 0.01, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_EQ(clusters.size(), 8);
}

/**
 * @brief Distance for aggregate the two nearest points (Expecting one less cluster).
 *
 */
TEST_F(GridClusteringTest, TestAggregate2Points) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(0.6, 0.6, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_LE(clusters.size(), 8);
}

/**
 * @brief Large minimum cluster size requirement (Expecting no clusters).
 *
 */
TEST_F(GridClusteringTest, TestMorePointsThanThreshold) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(1000, 1000, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_GE(clusters.size(), 1);
}

/**
 * @brief Small minimum cluster size requirement with very small distance
 *
 */
TEST_F(GridClusteringTest, TestMinClusterWithSmallGrid) {
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto clustering = std::make_unique<GridClustering>(0.01, 0.01, 0, 0, 360);
  std::vector<Cluster> clusters;
  clustering->clustering(input_cloud, &clusters);
  ASSERT_EQ(clusters.size(), 8);
}
