#include <gtest/gtest.h>

#include "clustering/dbscan.hpp"
#include "cone_validator/deviation_validator.hpp"

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

/**
 * @brief Fixture for testing the DeviationValidator class.
 */
class StandardDeviationTest : public ::testing::Test {
public:
  /**
   * @brief Set up the test fixtures.
   */
  void SetUp() override { _point_cloud_ptr_ = nullptr; }
  // We will create stack instances in each test.
  pcl::PointCloud<pcl::PointXYZI>* _point_cloud_ptr_;
  Plane _plane_;
  EvaluatorResults results;
};

/**
 * @brief Test case to validate a cluster with zero Z standard deviation.
 */
TEST_F(StandardDeviationTest, ZeroZDeviation) {
  // Create a stack-allocated point cloud.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  (void)cloud.emplace_back(1.0, 2.0, 10, 0.1);
  (void)cloud.emplace_back(4.0, 5.0, 10, 0.2);
  (void)cloud.emplace_back(7.0, 8.0, 10, 0.3);
  // Wrap the stack object with a non-owning shared pointer.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  // Create cluster using the wrapped point cloud.
  auto cluster = Cluster(cloud_ptr);
  const auto deviation_validator = DeviationValidator(-1, 100, 0.1, 100);

  deviation_validator.coneValidator(&cluster, &results, _plane_);
  ASSERT_NEAR(results.deviation_xoy, 1.0, 1e-6);
  ASSERT_LT(results.deviation_z, 1.0);
  ASSERT_GE(results.deviation_z, 0.0);
}

/**
 * @brief Test case to validate a cluster with non-zero Z standard deviation.
 */
TEST_F(StandardDeviationTest, NonZeroZDeviation) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  (void)cloud.emplace_back(0.0, 0.0, 0.8, 0.8);
  (void)cloud.emplace_back(0.0, 0.0, 0.1, 0.1);
  (void)cloud.emplace_back(0.0, 0.0, 0.3, 0.3);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  auto cluster = Cluster(cloud_ptr);
  const auto deviation_validator = DeviationValidator(-1, 100, 0.1, 100);

  deviation_validator.coneValidator(&cluster, &results, _plane_);
  ASSERT_NEAR(results.deviation_xoy, 1.0, 1e-6);
  ASSERT_NEAR(results.deviation_z, 1.0, 1e-6);
}

/**
 * @brief Test case to validate a cluster with zero XoY standard deviation.
 */
TEST_F(StandardDeviationTest, ZeroXoYDeviation) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  (void)cloud.emplace_back(1.0, 2.0, 10, 0.1);
  (void)cloud.emplace_back(1.0, 2.0, 100, 0.2);
  (void)cloud.emplace_back(1.0, 2.0, 100, 0.3);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  auto cluster = Cluster(cloud_ptr);
  const auto deviation_validator = DeviationValidator(0.1, 100, -1, 100);

  deviation_validator.coneValidator(&cluster, &results, _plane_);
  ASSERT_LT(results.deviation_xoy, 1.0);
  ASSERT_GE(results.deviation_xoy, 0.0);
  ASSERT_NEAR(results.deviation_z, 1.0, 1e-6);
}

/**
 * @brief Test case to validate a cluster with non-zero XoY standard deviation.
 */
TEST_F(StandardDeviationTest, NonZeroXoYDeviation) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  (void)cloud.emplace_back(1.0, 2.0, 10, 0.1);
  (void)cloud.emplace_back(3.0, 5.0, 100, 0.2);
  (void)cloud.emplace_back(10.0, -6.0, 100, 0.3);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  auto cluster = Cluster(cloud_ptr);
  const auto deviation_validator = DeviationValidator(0.1, 100, -1, 100);

  deviation_validator.coneValidator(&cluster, &results, _plane_);
  ASSERT_NEAR(results.deviation_xoy, 1.0, 1e-6);
  ASSERT_NEAR(results.deviation_z, 1.0, 1e-6);
}

/**
 * @brief Test case to validate the a cluster with zero xOy and Z standard deviation.
 */
TEST_F(StandardDeviationTest, ZeroXoYAndZDeviation) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  (void)cloud.emplace_back(1.0, 2.0, 10, 0.1);
  (void)cloud.emplace_back(1.0, 2.0, 10, 0.2);
  (void)cloud.emplace_back(1.0, 2.0, 10, 0.3);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  auto cluster = Cluster(cloud_ptr);
  const auto deviation_validator = DeviationValidator(0.1, 100, 0.1, 100);

  deviation_validator.coneValidator(&cluster, &results, _plane_);
  ASSERT_LT(results.deviation_xoy, 1.0);
  ASSERT_GE(results.deviation_xoy, 0.0);
  ASSERT_LT(results.deviation_z, 1.0);
  ASSERT_GE(results.deviation_z, 0.0);
}

/**
 * @brief Test case to validate the a cluster with non-zero xOy and Z standard deviation.
 */
TEST_F(StandardDeviationTest, NonZeroXoYAndZDeviation) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  (void)cloud.emplace_back(1.0, 2.0, 0.8, 0.1);
  (void)cloud.emplace_back(3.0, 5.0, 0.1, 0.2);
  (void)cloud.emplace_back(10.0, -6.0, 0.3, 0.3);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  auto cluster = Cluster(cloud_ptr);
  const auto deviation_validator = DeviationValidator(0.1, 100, 0.1, 100);

  deviation_validator.coneValidator(&cluster, &results, _plane_);
  ASSERT_NEAR(results.deviation_xoy, 1.0, 1e-6);
  ASSERT_NEAR(results.deviation_z, 1.0, 1e-6);
}
