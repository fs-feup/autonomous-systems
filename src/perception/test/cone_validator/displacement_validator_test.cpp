#include <gtest/gtest.h>

#include <cone_validator/displacement_validator.hpp>

/**
 * @brief Test fixture for DisplacementValidator class.
 */
class DisplacementValidatorTest : public ::testing::Test {
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
 * @brief Test case to validate if the cluster is well-distributed across all axes.
 */
TEST_F(DisplacementValidatorTest, ClusterWithWellDistributedPoints) {
  const DisplacementValidator validator(0.1, 0.1, 0.25);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.0, 0.0, 0.0, 0);
  (void)point_cloud->points.emplace_back(0.3, 0.7, 0.5, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
  ASSERT_DOUBLE_EQ(result[1], 1.0);
  ASSERT_DOUBLE_EQ(result[2], 1.0);
}

/**
 * @brief Test case to validate points below the minimum distance on the x-axis.
 */
TEST_F(DisplacementValidatorTest, BelowThresholdOnXAxis) {
  const DisplacementValidator validator(0.1, 0.1, 0.25);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.3, 0.7, 0.1, 0);
  (void)point_cloud->points.emplace_back(0.34, 0.5, 0.7, 0);
  (void)point_cloud->points.emplace_back(0.28, 0.0, 0.5, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_LT(result[0], 1.0);
  ASSERT_GE(result[0], 0.0);
  ASSERT_DOUBLE_EQ(result[1], 1.0);
  ASSERT_DOUBLE_EQ(result[2], 1.0);
}

/**
 * @brief Test case to validate points below the minimum distance on the y-axis.
 */
TEST_F(DisplacementValidatorTest, BelowThresholdOnYAxis) {
  const DisplacementValidator validator(0.1, 0.1, 0.25);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.3, 0.3, 0.1, 0);
  (void)point_cloud->points.emplace_back(0.7, 0.32, 0.7, 0);
  (void)point_cloud->points.emplace_back(0.5, 0.34, 0.5, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
  ASSERT_LT(result[1], 1.0);
  ASSERT_GE(result[1], 0.0);
  ASSERT_DOUBLE_EQ(result[2], 1.0);
}

/**
 * @brief Test case to validate points below the minimum distance on the z-axis.
 */
TEST_F(DisplacementValidatorTest, BelowThresholdOnZAxis) {
  const DisplacementValidator validator(0.1, 0.1, 0.25);

  const auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
  (void)point_cloud->points.emplace_back(0.3, 0.7, 0.1, 0);
  (void)point_cloud->points.emplace_back(0.6, 0.5, 0.17, 0);
  (void)point_cloud->points.emplace_back(0.9, 0.0, 0.08, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
  ASSERT_DOUBLE_EQ(result[1], 1.0);
  ASSERT_LT(result[2], 1.0);
  ASSERT_GE(result[2], 0.0);
}
