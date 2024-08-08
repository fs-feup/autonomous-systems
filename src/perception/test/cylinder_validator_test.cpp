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
 * @brief Test case to validate if a point is inside the cylinder.
 */
TEST_F(CylinderValidatorTest, PointInsideCylinder) {
  CylinderValidator validator(0.5, 1.0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  // Validate point against the cylinder
  bool result = validator.cone_validator(&cylinderPointCloud, plane);

  ASSERT_TRUE(result);
}

/**
 * @brief Test case to validate if a point is outside the cylinder.
 */
TEST_F(CylinderValidatorTest, PointOutsideCylinder) {
  CylinderValidator validator(0.5, 1.0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud->points.push_back(pcl::PointXYZI{1.0, 1.0, 2.0, 0});
  point_cloud->points.push_back(pcl::PointXYZI{20.0, 20.0, 20.0, 0});

  Cluster cylinderPointCloud = Cluster(point_cloud);

  bool result = validator.cone_validator(&cylinderPointCloud, plane);

  ASSERT_FALSE(result);
}
