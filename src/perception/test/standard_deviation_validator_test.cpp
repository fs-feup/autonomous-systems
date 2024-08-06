#include "clustering/dbscan.hpp"
#include "cone_validator/deviation_validator.hpp"

#include <gtest/gtest.h>


/**
 * @brief Fixture for testing the DeviationValidator class.
 */
class StandardDeviationTest : public ::testing::Test {
 public:
   /**
   * @brief Set up the test fixtures.
   */
  void SetUp() override {
    _point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud_;
  Plane _plane_;
};

/**
 * @brief Test case to validate the an empty cluster
 */
TEST_F(StandardDeviationTest, NullPointCloud) {
  auto deviation_validator = DeviationValidator(-1, 100, -1, 100);
  auto cluster = Cluster(_point_cloud_);

  ASSERT_FALSE(deviation_validator.coneValidator(&cluster, _plane_));
}

/**
 * @brief Test case to validate the a cluster with the same Z (0 standard deviation)
 */
TEST_F(StandardDeviationTest, ZeroZDeviation) {
  auto deviation_validator = DeviationValidator(-1, 100, 0.1, 100);
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  _point_cloud_->push_back(pcl::PointXYZI(4.0, 5.0, 10, 0.2));
  _point_cloud_->push_back(pcl::PointXYZI(7.0, 8.0, 10, 0.3));
  auto cluster = Cluster(_point_cloud_);

  ASSERT_FALSE(deviation_validator.coneValidator(&cluster, _plane_));
}

/**
 * @brief Test case to validate the a cluster with different Z (non 0 standard deviation)
 */
TEST_F(StandardDeviationTest, NonZeroZDeviation) {
  auto deviation_validator = DeviationValidator(-1, 100, 0.1, 100);
  _point_cloud_->push_back(pcl::PointXYZI(0.0, 0.0, 0.8, 0.8));
  _point_cloud_->push_back(pcl::PointXYZI(0.0, 0.0, 0.1, 0.1));
  _point_cloud_->push_back(pcl::PointXYZI(0.0, 0.0, 0.3, 0.3));
  auto cluster = Cluster(_point_cloud_);

  ASSERT_TRUE(deviation_validator.coneValidator(&cluster, _plane_));
}

/**
 * @brief Test case to validate the a cluster with the same xOy (0 standard deviation)
 */
TEST_F(StandardDeviationTest, ZeroXoYDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, -1, 100);
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 100, 0.2));
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 100, 0.3));
  auto cluster = Cluster(_point_cloud_);

  ASSERT_FALSE(deviation_validator.coneValidator(&cluster, _plane_));
}

/**
 * @brief Test case to validate the a cluster with different xOy
 */
TEST_F(StandardDeviationTest, NonZeroXoYDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, -1, 100);
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  _point_cloud_->push_back(pcl::PointXYZI(3.0, 5.0, 100, 0.2));
  _point_cloud_->push_back(pcl::PointXYZI(10.0, -6.0, 100, 0.3));
  auto cluster = Cluster(_point_cloud_);

  ASSERT_TRUE(deviation_validator.coneValidator(&cluster, _plane_));
}

/**
 * @brief Test case to validate the a cluster with zero xOy and Z standard deviation
 */
TEST_F(StandardDeviationTest, ZeroXoYAndZDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, 0.1, 100);
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.1));
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.2));
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 10, 0.3));
  auto cluster = Cluster(_point_cloud_);

  ASSERT_FALSE(deviation_validator.coneValidator(&cluster, _plane_));
}

/**
 * @brief Test case to validate the a cluster with non-zero xOy and Z standar deviation
 */
TEST_F(StandardDeviationTest, NonZeroXoYAndZDeviation) {
  auto deviation_validator = DeviationValidator(0.1, 100, 0.1, 100);
  _point_cloud_->push_back(pcl::PointXYZI(1.0, 2.0, 0.8, 0.1));
  _point_cloud_->push_back(pcl::PointXYZI(3.0, 5.0, 0.1, 0.2));
  _point_cloud_->push_back(pcl::PointXYZI(10.0, -6.0, 0.3, 0.3));
  auto cluster = Cluster(_point_cloud_);

  ASSERT_TRUE(deviation_validator.coneValidator(&cluster, _plane_));
}