#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_validator/cylinder_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test fixture for CylinderValidator class.
 */
class CylinderValidatorTest : public ::testing::Test {
protected:
  /**
   * @brief Set up function to initialize.
   */
  void SetUp() override { plane = Plane(1, 0, 0, 0); }
  Plane plane;  ///< Plane object for testing.
};

/**
 * @brief Test case to validate if points are inside the small cylinder.
 */
TEST_F(CylinderValidatorTest, PointsInsideSmallCylinder) {
  CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  ASSERT_NEAR(result[0], 1.0, 1e-6);  // Distance XY ratio is 1.
  ASSERT_NEAR(result[0], 1.0, 1e-6);  // Distance Z ratio is 1.
  ASSERT_DOUBLE_EQ(result[2], 1.0);   // All points are inside the cylinder.
}

/**
 * @brief Test case to validate if points are inside the large cylinder.
 */
TEST_F(CylinderValidatorTest, PointsInsideLargeCylinder) {
  CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{0.6, 0.6, 1.2, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  // Simulate height validator and set cluster as large.
  cylinderPointCloud.set_is_large();

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  ASSERT_NEAR(result[0], 1.0, 1e-6);  // Distance XY ratio is 1.
  ASSERT_NEAR(result[0], 1.0, 1e-6);  // Distance Z ratio is 1.
  ASSERT_DOUBLE_EQ(result[2], 1.0);   // All points are inside the cylinder.
}

/**
 * @brief Test case to validate if points are outside all cylinders.
 */
TEST_F(CylinderValidatorTest, PointsFarOutsideCylinders) {
  CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{1.0, 1.0, 2.0, 0});
  point_cloud->points.push_back(pcl::PointXYZI{20.0, 20.0, 20.0, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  ASSERT_NEAR(result[0], 0.0, 1e-6);  // Distance XY ratio is 0.
  ASSERT_NEAR(result[0], 0.0, 1e-6);  // Distance XY ratio is 0.
  ASSERT_LT(result[2], 1.0);          // Some points are outside the cylinder.
}

/**
 * @brief Test case to validate if points are outside all cylinders.
 */
TEST_F(CylinderValidatorTest, PointsOutsideCylinders) {
  CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{1.0, 1.0, 2.0, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.6, 0.6, 0.9, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  ASSERT_GE(result[0], 0.5);  // Distance XY ratio is higher than cap.
  ASSERT_LE(result[0], 1.0);  // Distance XY ratio is capped at 1.
  ASSERT_GE(result[1], 0.5);  // Distance Z ratio is higher than cap.
  ASSERT_LE(result[1], 1.0);  // Distance Z ratio is capped at 1.
  ASSERT_LT(result[2], 1.0);  // Some points are outside the cylinder.
}