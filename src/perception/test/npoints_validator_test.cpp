#include <gtest/gtest.h>
#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_validator/npoints_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test fixture for HeightValidator class.
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
 * @brief Test case to validate if the cone has lower number of points than the minimum.
 */
TEST_F(NPointsValidatorTest, ConeWithSmallNumberOfPoints) {
  NPointsValidator validator = NPointsValidator(4);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.emplace_back(0.3, 0.0, 0, 0);

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_FALSE(result);
}

/**
 * @brief Test case to validate if the cone has higher number of points than the minimum.
 */
TEST_F(NPointsValidatorTest, ConeWithHighNumberOfPoints) {
  NPointsValidator validator = NPointsValidator(4);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.insert(point_cloud->points.end(), {{0.3, 0.0, 0.0, 0},
                                                         {0.5, 0.1, 0.2, 0},
                                                         {0.1, 0.2, 0.3, 0},
                                                         {-0.2, -0.3, 0.1, 0},
                                                         {0.4, -0.1, 0.0, 0},
                                                         {-0.1, 0.5, 0.2, 0},
                                                         {0.0, -0.4, 0.3, 0},
                                                         {0.3, 0.3, -0.2, 0},
                                                         {-0.3, 0.1, -0.1, 0},
                                                         {0.2, -0.2, 0.4, 0}});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_TRUE(result);
}

/**
 * @brief Test case to validate if the cone has the same number of points as the minimum.
 */
TEST_F(NPointsValidatorTest, ConeWithMinNumberOfPoints) {
  NPointsValidator validator = NPointsValidator(4);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.insert(point_cloud->points.end(), {
                                                            {0.3, 0.0, 0.0, 0},
                                                            {0.5, 0.1, 0.2, 0},
                                                            {0.1, 0.2, 0.3, 0},
                                                            {-0.2, -0.3, 0.1, 0},
                                                        });

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_TRUE(result);
}