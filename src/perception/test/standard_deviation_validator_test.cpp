#include "clustering/dbscan.hpp"
#include "cone_validator/deviation_validator.hpp"

#include <gtest/gtest.h>


/**
 * @brief Fixture for testing the DeviationValidator class.
 */
class StandardDeviationTest : public ::testing::Test {
 protected:
   /**
   * @brief Set up the test fixtures.
   */
  void SetUp() override {
    point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
  Plane plane;
};

/**
 * @brief Test case to validate the an empty cluster
 */
TEST_F(StandardDeviationTest, NullPointCloud) {
  auto deviation_validator = new DeviationValidator(-1, 100, -1, 100);
  auto cluster = new Cluster(point_cloud);

  ASSERT_FALSE(deviation_validator->coneValidator(cluster, plane));
}

/**
 * @brief Test case to validate the a cluster with the same Z
 */
TEST_F(StandardDeviationTest, ZeroZDeviation) {
  auto deviation_validator = new DeviationValidator(-1, 100, 0.1, 100);
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  point_cloud->push_back(pcl::PointXYZI(4.0, 5.0, 10, 0.2));
  point_cloud->push_back(pcl::PointXYZI(7.0, 8.0, 10, 0.3));
  auto cluster = new Cluster(point_cloud);

  ASSERT_FALSE(deviation_validator->coneValidator(cluster, plane));
}

/**
 * @brief Test case to validate the a cluster with different Z
 */
TEST_F(StandardDeviationTest, NonZeroZDeviation) {
  auto deviation_validator = new DeviationValidator(-1, 100, 0.1, 100);
  point_cloud->push_back(pcl::PointXYZI(0.0, 0.0, 0.8, 0.8));
  point_cloud->push_back(pcl::PointXYZI(0.0, 0.0, 0.1, 0.1));
  point_cloud->push_back(pcl::PointXYZI(0.0, 0.0, 0.3, 0.3));
  auto cluster = new Cluster(point_cloud);

  ASSERT_TRUE(deviation_validator->coneValidator(cluster, plane));
}

/**
 * @brief Test case to validate the a cluster with the same xOy
 */
TEST_F(StandardDeviationTest, ZeroXoYDeviation) {
  auto deviation_validator = new DeviationValidator(0.1, 100, -1, 100);
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 100, 0.2));
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 100, 0.3));
  auto cluster = new Cluster(point_cloud);

  ASSERT_FALSE(deviation_validator->coneValidator(cluster, plane));
}

/**
 * @brief Test case to validate the a cluster with different xOy
 */
TEST_F(StandardDeviationTest, NonZeroXoYDeviation) {
  auto deviation_validator = new DeviationValidator(0.1, 100, -1, 100);
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  point_cloud->push_back(pcl::PointXYZI(3.0, 5.0, 100, 0.2));
  point_cloud->push_back(pcl::PointXYZI(10.0, -6.0, 100, 0.3));
  auto cluster = new Cluster(point_cloud);

  ASSERT_TRUE(deviation_validator->coneValidator(cluster, plane));
}

/**
 * @brief Test case to validate the a cluster with zero xOy and Z
 */
TEST_F(StandardDeviationTest, ZeroXoYAndZDeviation) {
  auto deviation_validator = new DeviationValidator(0.1, 100, 0.1, 100);
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.2));
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.3));
  auto cluster = new Cluster(point_cloud);

  ASSERT_FALSE(deviation_validator->coneValidator(cluster, plane));
}

/**
 * @brief Test case to validate the a cluster with non-zero xOy and Z
 */
TEST_F(StandardDeviationTest, NonZeroXoYAndZDeviation) {
  auto deviation_validator = new DeviationValidator(0.1, 100, 0.1, 100);
  point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 0.8, 0.1));
  point_cloud->push_back(pcl::PointXYZI(3.0, 5.0, 0.1, 0.2));
  point_cloud->push_back(pcl::PointXYZI(10.0, -6.0, 0.3, 0.3));
  auto cluster = new Cluster(point_cloud);

  ASSERT_TRUE(deviation_validator->coneValidator(cluster, plane));
}