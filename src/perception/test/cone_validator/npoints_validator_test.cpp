#include <gtest/gtest.h>
#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_validator/npoints_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test fixture for NPointsValidator class.
 */
class NPointsValidatorTest : public ::testing::Test {
protected:
  /**
   * @brief Set up function to initialize a plane.
   */
  void SetUp() override { plane = Plane(1, 0, 0, 0); }

  Plane plane;  ///< Plane object for testing.
};

/**
 * @brief Test case to validate a cluster with fewer points than the minimum threshold.
 */
TEST_F(NPointsValidatorTest, ConeWithFewerPointsThanThreshold) {
  const NPointsValidator validator(4);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.emplace_back(0.3, 0.0, 0.0, 0);  // 1 point

  Cluster cone_point_cloud(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_LT(result[0], 1.0);
}

/**
 * @brief Test case to validate a cluster with exactly the minimum threshold number of points.
 */
TEST_F(NPointsValidatorTest, ConeWithExactMinPoints) {
  const NPointsValidator validator(4);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.insert(point_cloud->points.end(), {{0.3, 0.0, 0.0, 0},
                                                               {0.5, 0.1, 0.2, 0},
                                                               {0.1, 0.2, 0.3, 0},
                                                               {-0.2, -0.3, 0.1, 0}});  // 4 points

  Cluster cone_point_cloud(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
}

/**
 * @brief Test case to validate a cluster with more points than the minimum threshold.
 */
TEST_F(NPointsValidatorTest, ConeWithMorePointsThanThreshold) {
  const NPointsValidator validator(4);

  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  (void)point_cloud->points.insert(point_cloud->points.end(), {{0.3, 0.0, 0.0, 0},
                                                               {0.5, 0.1, 0.2, 0},
                                                               {0.1, 0.2, 0.3, 0},
                                                               {-0.2, -0.3, 0.1, 0},
                                                               {0.4, -0.1, 0.0, 0}});  // 5 points

  Cluster cone_point_cloud(point_cloud);

  std::vector<double> result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_DOUBLE_EQ(result[0], 1.0);
}