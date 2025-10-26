#include <gtest/gtest.h>
#include <cone_validator/height_validator.hpp>

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
};

/**
 * @brief Test case to validate if the cone height is within the small cone height threshold.
 */
TEST_F(HeightValidatorTest, ConeWithinSmallHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.3, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
  ASSERT_NEAR(result[1], 0.8, 1e-6);  // 0.3/0.375
  ASSERT_FALSE(cone_point_cloud.get_is_large());
}

/**
 * @brief Test case to validate if the cone height exceeds the small cone threshold
 * but is within the large cone height threshold.
 */
TEST_F(HeightValidatorTest, ConeWithinLargeHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.4, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
  ASSERT_NEAR(result[1], 0.8, 1e-6);  // 0.4/0.5
  ASSERT_TRUE(cone_point_cloud.get_is_large());
}

/**
 * @brief Test case to validate if the cone height exceeds all thresholds.
 */
TEST_F(HeightValidatorTest, ConeExceedsHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.6, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_LT(result[0], 1.0);
  ASSERT_GE(result[0], 0.0);
  ASSERT_DOUBLE_EQ(result[1], 0.0);
  ASSERT_TRUE(cone_point_cloud.get_is_large());
}

/**
 * @brief Test case to validate if the cone height is below the minimum height threshold.
 */
TEST_F(HeightValidatorTest, ConeBelowHeightThreshold) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.03, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 0.0);  // Lower than cap
  ASSERT_DOUBLE_EQ(result[1], 0.0);
  ASSERT_FALSE(cone_point_cloud.get_is_large());
}

/**
 * @brief Test case to validate if the cone height is near the minimum threshold but valid.
 */
TEST_F(HeightValidatorTest, ConeNearMinimumHeight) {
  const HeightValidator validator(0.1, 0.5, 0.375, 0.5);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.15, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
  ASSERT_NEAR(result[1], 0.4, 1e-6);  // 0.15/0.375
  ASSERT_FALSE(cone_point_cloud.get_is_large());
}
