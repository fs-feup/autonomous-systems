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
   * @brief Set up function to initialize a plane.
   */
  void SetUp() override { plane = Plane(1, 0, 0, 0); }

  Plane plane;  ///< Plane object for testing.
};

/**
 * @brief Test case to validate if a point is inside the small cylinder.
 */
TEST_F(CylinderValidatorTest, PointInsideSmallCylinder) {
  CylinderValidator validator(0.5, 1.0, 0.7, 1.5);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  // Validate point against the cylinder
  bool result = validator.coneValidator(&cylinderPointCloud, plane);

  ASSERT_TRUE(result);
  ASSERT_FALSE(cylinderPointCloud.get_is_large());
}

/**
 * @brief Test case to validate if a point is inside the large cylinder.
 */
TEST_F(CylinderValidatorTest, PointInsideLargeCylinder) {
  CylinderValidator validator(0.5, 1.0, 0.7, 1.5);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{0.6, 0.6, 1.2, 0});
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  // Simulate height validator and set cluster as large.
  cylinderPointCloud.set_is_large();

  // Validate point against the cylinder
  bool result = validator.coneValidator(&cylinderPointCloud, plane);

  ASSERT_TRUE(result);
  ASSERT_TRUE(cylinderPointCloud.get_is_large());
}

/**
 * @brief Test case to validate if a point is outside the all cylinders.
 */
TEST_F(CylinderValidatorTest, PointOutsideCylinder) {
  CylinderValidator validator(0.5, 1.0, 0.7, 1.5);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{1.0, 1.0, 2.0, 0});
  point_cloud->points.push_back(pcl::PointXYZI{20.0, 20.0, 20.0, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  bool result = validator.coneValidator(&cylinderPointCloud, plane);

  ASSERT_FALSE(result);
}
