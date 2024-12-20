#include <gtest/gtest.h>
#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_validator/displacement_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test fixture for HeightValidator class.
 */
class DisplacementValidatorTest : public ::testing::Test {
protected:
  /**
   * @brief Set up function to initialize a plane.
   */
  void SetUp() override { plane = Plane(1, 0, 0, 0); }

  Plane plane;  ///< Plane object for testing.
};

/**
 * @brief Test case to validate if the distance between points in the
 * cluster is above the minimum value in all axis.
 */
TEST_F(DisplacementValidatorTest, clusterWithWellDistancedPoints) {
  DisplacementValidator validator = DisplacementValidator(0.1, 0.1, 0.25);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.7, 0.5, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.0, 0.3, 0.0, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_TRUE(result);
}

/**
 * @brief Test case to validate if the points are below the minimum
 * distance only on the x axis.
 */
TEST_F(DisplacementValidatorTest, belowOnXAxis) {
  DisplacementValidator validator = DisplacementValidator(0.1, 0.1, 0.25);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.7, 0.1, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.34, 0.5, 0.7, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.28, 0.0, 0.5, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_FALSE(result);
}

/**
 * @brief Test case to validate if the points are below the minimum
 * distance only on the y axis.
 */
TEST_F(DisplacementValidatorTest, belowOnYAxis) {
  DisplacementValidator validator = DisplacementValidator(0.1, 0.1, 0.25);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.1, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.7, 0.32, 0.7, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.5, 0.34, 0.5, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_FALSE(result);
}

/**
 * @brief Test case to validate if the points are below the minimum
 * distance only on the z axis.
 */
TEST_F(DisplacementValidatorTest, belowOnZAxis) {
  DisplacementValidator validator = DisplacementValidator(0.1, 0.1, 0.25);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.7, 0.1, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.6, 0.5, 0.17, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.9, 0.0, 0.08, 0});

  Cluster cone_point_cloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cone_point_cloud, plane);

  ASSERT_FALSE(result);
}