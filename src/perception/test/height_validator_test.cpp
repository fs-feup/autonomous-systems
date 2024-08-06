#include <gtest/gtest.h>
#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_validator/height_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test fixture for HeightValidator class.
 */
class HeightValidatorTest : public ::testing::Test {
 protected:
  /**
   * @brief Set up function to initialize a plane.
   */
  void SetUp() override { plane = Plane(1, 0, 0, 0); }

  Plane plane;  ///< Plane object for testing.
};

/**
 * @brief Test case to validate if the cone height is within the height threshold.
 */
TEST_F(HeightValidatorTest, ConeWithinHeightThreshold) {
  HeightValidator validator = HeightValidator(0.1, 0.375);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.emplace_back(pcl::PointXYZI{0.3, 0.0, 0, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_TRUE(result);
}

/**
 * @brief Test case to validate if the cone height exceeds the height threshold.
 */
TEST_F(HeightValidatorTest, ConeExceedsHeightThreshold) {
  HeightValidator validator = HeightValidator(0.1, 0.375);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.emplace_back(pcl::PointXYZI{1.0, 0.0, 0, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_FALSE(result);
}

/**
 * @brief Test case to validate if the cone height is below the minimum height threshold.
 */
TEST_F(HeightValidatorTest, ConeBelowHeightThreshold) {
  HeightValidator validator = HeightValidator(0.1, 0.375);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.emplace_back(pcl::PointXYZI{0.01, 0.0, 0, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_FALSE(result);
}

/**
 * @brief Test case to validate if the cone height isn't below the minimum height threshold.
 */
TEST_F(HeightValidatorTest, ConeHeightThreshold) {
  HeightValidator validator = HeightValidator(0.1, 0.375);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.emplace_back(pcl::PointXYZI{0.15, 0.0, 0, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_TRUE(result);
}
