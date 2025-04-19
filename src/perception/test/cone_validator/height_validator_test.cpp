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
  void SetUp() override {
    plane = Plane(0, 0, 1, 0);  // Horizontal plane (z=0)
  }

  Plane plane;  ///< Plane object for testing.
  EvaluatorResults results;
};

/**
 * @brief Test case to validate if the cone height is within the small cone height threshold.
 */
TEST_F(HeightValidatorTest, ConeWithinSmallHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.3, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  validator.coneValidator(&cone_point_cloud, &results, plane);

  ASSERT_DOUBLE_EQ(results.height_out_ratio_small, 1.0);
  ASSERT_NEAR(results.height_in_ratio_small, 0.8, 1e-6);  // 0.3/0.375
  ASSERT_FALSE(results.height_large);
}

/**
 * @brief Test case to validate if the cone height exceeds the small cone threshold
 * but is within the large cone height threshold.
 */
TEST_F(HeightValidatorTest, ConeWithinLargeHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.4, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  validator.coneValidator(&cone_point_cloud, &results, plane);

  ASSERT_DOUBLE_EQ(results.height_out_ratio_large, 1.0);
  ASSERT_NEAR(results.height_in_ratio_large, 0.8, 1e-6);  // 0.4/0.5
  ASSERT_TRUE(results.height_large);
}

/**
 * @brief Test case to validate if the cone height exceeds all thresholds.
 */
TEST_F(HeightValidatorTest, ConeExceedsHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.6, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  validator.coneValidator(&cone_point_cloud, &results, plane);

  ASSERT_LT(results.height_out_ratio_small, 1.0);
  ASSERT_GE(results.height_out_ratio_small, 0.0);
  ASSERT_DOUBLE_EQ(results.height_in_ratio_small, 0.0);
  ASSERT_TRUE(results.height_large);
}

/**
 * @brief Test case to validate if the cone height is below the minimum height threshold.
 */
TEST_F(HeightValidatorTest, ConeBelowHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.03, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  validator.coneValidator(&cone_point_cloud, &results, plane);

  ASSERT_DOUBLE_EQ(results.height_out_ratio_small, 0.0);  // Lower than cap
  ASSERT_DOUBLE_EQ(results.height_in_ratio_small, 0.0);
  ASSERT_FALSE(results.height_large);
}

/**
 * @brief Test case to validate if the cone height is near the minimum threshold but valid.
 */
TEST_F(HeightValidatorTest, ConeNearMinimumHeight) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.15, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  validator.coneValidator(&cone_point_cloud, &results, plane);

  ASSERT_DOUBLE_EQ(results.height_out_ratio_small, 1.0);
  ASSERT_NEAR(results.height_in_ratio_small, 0.4, 1e-6);  // 0.15/0.375
  ASSERT_FALSE(results.height_large);
}
