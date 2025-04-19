#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_validator/cylinder_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/evaluator_results.hpp>
#include <utils/plane.hpp>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

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
  EvaluatorResults results;
};

/**
 * @brief Test case to validate if points are inside the small cylinder.
 */
TEST_F(CylinderValidatorTest, PointsInsideSmallCylinder) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  // Create a stack-allocated point cloud.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});
  // Wrap the stack object with a non-owning shared pointer.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  Cluster cylinderPointCloud(cloud_ptr);

  validator.coneValidator(&cylinderPointCloud, &results, plane);

  // Check the results.
  ASSERT_NEAR(results.cylinder_out_distance_xy_small, 1.0, 1e-6);  // Distance XY ratio is 1.
  ASSERT_NEAR(results.cylinder_out_distance_z_small, 1.0, 1e-6);   // Distance Z ratio is 1.
  ASSERT_DOUBLE_EQ(results.cylinder_n_out_points + results.cylinder_n_large_points,
                   0.0);  // All points are inside the cylinder.
}

/**
 * @brief Test case to validate if points are inside the large cylinder.
 */
TEST_F(CylinderValidatorTest, PointsInsideLargeCylinder) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.points.push_back(pcl::PointXYZI{0.6, 0.6, 1.2, 0});
  cloud.points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  Cluster cylinderPointCloud(cloud_ptr);

  validator.coneValidator(&cylinderPointCloud, &results, plane);

  ASSERT_NEAR(results.cylinder_out_distance_xy_large, 1.0, 1e-6);  // Distance XY ratio is 1.
  ASSERT_NEAR(results.cylinder_out_distance_z_large, 1.0, 1e-6);   // Distance Z ratio is 1.
  ASSERT_DOUBLE_EQ(results.cylinder_n_out_points,
                   0.0);  // All points are inside the cylinder.
}

/**
 * @brief Test case to validate if points are outside all cylinders.
 */
TEST_F(CylinderValidatorTest, PointsFarOutsideCylinders) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.points.push_back(pcl::PointXYZI{1.0, 1.0, 2.0, 0});
  cloud.points.push_back(pcl::PointXYZI{20.0, 20.0, 20.0, 0});
  cloud.points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  Cluster cylinderPointCloud(cloud_ptr);

  validator.coneValidator(&cylinderPointCloud, &results, plane);

  ASSERT_NEAR(results.cylinder_out_distance_xy_small, 0.0, 1e-6);  // Distance XY ratio is 1.
  ASSERT_NEAR(results.cylinder_out_distance_z_small, 0.0, 1e-6);   // Distance Z ratio is 1.
  ASSERT_NEAR(results.cylinder_out_distance_xy_large, 0.0, 1e-6);  // Distance XY ratio is 1.
  ASSERT_NEAR(results.cylinder_out_distance_z_large, 0.0, 1e-6);   // Distance Z ratio is 1.
  ASSERT_EQ(results.cylinder_n_out_points + results.cylinder_n_large_points,
            3.0);  // All points are inside the cylinder.
}

/**
 * @brief Test case to validate if points are outside all cylinders.
 */
TEST_F(CylinderValidatorTest, PointsOutsideCylinders) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.points.push_back(pcl::PointXYZI{1.0, 1.0, 2.0, 0});
  cloud.points.push_back(pcl::PointXYZI{0.6, 0.6, 0.9, 0});
  cloud.points.push_back(pcl::PointXYZI{0.3, 0.3, 0.5, 0});
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  Cluster cylinderPointCloud(cloud_ptr);

  validator.coneValidator(&cylinderPointCloud, &results, plane);

  ASSERT_GE(results.cylinder_out_distance_xy_small, 0.5);  // Distance XY ratio is higher than cap.
  ASSERT_LE(results.cylinder_out_distance_xy_small, 1.0);  // Distance XY ratio is capped at 1.
  ASSERT_GE(results.cylinder_out_distance_z_small, 0.5);   // Distance Z ratio is higher than cap.
  ASSERT_LE(results.cylinder_out_distance_z_small, 1.0);   // Distance Z ratio is capped at 1.
  ASSERT_GE(results.cylinder_n_large_points + results.cylinder_n_out_points,
            0.0);  // Some points are outside the cylinder.
}
