#include <gtest/gtest.h>

#include "clustering/dbscan.hpp"
#include "cone_validator/deviation_validator.hpp"

/**
 * @brief Fixture for testing the DeviationValidator class.
 */
class StandardDeviationTest : public ::testing::Test {
public:
  /**
   * @brief Set up the test fixtures.
   */
  void SetUp() override { _point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>); }
  pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud_;
  Plane _plane_;
};

/**
 * @brief Test case to validate a cluster with zero Z standard deviation.
 */
TEST_F(StandardDeviationTest, ZeroZDeviation) {
  auto deviation_validator = DeviationValidator(-1, 100, 0.1, 100);
  _point_cloud_->emplace_back(1.0, 2.0, 10, 0.1);
  _point_cloud_->emplace_back(4.0, 5.0, 10, 0.2);
  _point_cloud_->emplace_back(7.0, 8.0, 10, 0.3);
  auto cluster = Cluster(_point_cloud_);

  auto result = deviation_validator.coneValidator(&cluster, _plane_);
  EXPECT_EQ(result.size(), 2);
  ASSERT_NEAR(result[0], 1.0, 1e-6);
  ASSERT_LT(result[1], 1.0);
  ASSERT_GE(result[1], 0.0);
}

/**
 * @brief Test case to validate a cluster with non-zero Z standard deviation.
 */
TEST_F(StandardDeviationTest, NonZeroZDeviation) {
  auto deviation_validator = DeviationValidator(-1, 100, 0.1, 100);
  _point_cloud_->emplace_back(0.0, 0.0, 0.8, 0.8);
  _point_cloud_->emplace_back(0.0, 0.0, 0.1, 0.1);
  _point_cloud_->emplace_back(0.0, 0.0, 0.3, 0.3);
  auto cluster = Cluster(_point_cloud_);

  auto result = deviation_validator.coneValidator(&cluster, _plane_);
  EXPECT_EQ(result.size(), 2);
  ASSERT_NEAR(result[0], 1.0, 1e-6);
  ASSERT_NEAR(result[1], 1.0, 1e-6);
}

/**
 * @brief Test case to validate a cluster with zero XoY standard deviation.
 */
TEST_F(StandardDeviationTest, ZeroXoYDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, -1, 100);
  _point_cloud_->emplace_back(1.0, 2.0, 10, 0.1);
  _point_cloud_->emplace_back(1.0, 2.0, 100, 0.2);
  _point_cloud_->emplace_back(1.0, 2.0, 100, 0.3);
  auto cluster = Cluster(_point_cloud_);

  auto result = deviation_validator.coneValidator(&cluster, _plane_);
  EXPECT_EQ(result.size(), 2);
  ASSERT_LT(result[0], 1.0);
  ASSERT_GE(result[0], 0.0);
  ASSERT_NEAR(result[1], 1.0, 1e-6);
}

/**
 * @brief Test case to validate a cluster with non-zero XoY standard deviation.
 */
TEST_F(StandardDeviationTest, NonZeroXoYDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, -1, 100);
  _point_cloud_->emplace_back(1.0, 2.0, 10, 0.1);
  _point_cloud_->emplace_back(3.0, 5.0, 100, 0.2);
  _point_cloud_->emplace_back(10.0, -6.0, 100, 0.3);
  auto cluster = Cluster(_point_cloud_);

  auto result = deviation_validator.coneValidator(&cluster, _plane_);
  EXPECT_EQ(result.size(), 2);
  ASSERT_NEAR(result[0], 1.0, 1e-6);
  ASSERT_NEAR(result[1], 1.0, 1e-6);
}

/**
 * @brief Test case to validate the a cluster with zero xOy and Z standard deviation
 */
TEST_F(StandardDeviationTest, ZeroXoYAndZDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, 0.1, 100);
  _point_cloud_->emplace_back(1.0, 2.0, 10, 0.1);
  _point_cloud_->emplace_back(1.0, 2.0, 10, 0.2);
  _point_cloud_->emplace_back(1.0, 2.0, 10, 0.3);
  auto cluster = Cluster(_point_cloud_);

  auto result = deviation_validator.coneValidator(&cluster, _plane_);
  ASSERT_LT(result[0], 1.0);
  ASSERT_GE(result[0], 0.0);
  ASSERT_LT(result[1], 1.0);
  ASSERT_GE(result[1], 0.0);
}

/**
 * @brief Test case to validate the a cluster with non-zero xOy and Z standar deviation
 */
TEST_F(StandardDeviationTest, NonZeroXoYAndZDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, 0.1, 100);
  _point_cloud_->emplace_back(1.0, 2.0, 0.8, 0.1);
  _point_cloud_->emplace_back(3.0, 5.0, 0.1, 0.2);
  _point_cloud_->emplace_back(10.0, -6.0, 0.3, 0.3);
  auto cluster = Cluster(_point_cloud_);

  auto result = deviation_validator.coneValidator(&cluster, _plane_);
  ASSERT_NEAR(result[0], 1.0, 1e-6);
  ASSERT_NEAR(result[1], 1.0, 1e-6);
}