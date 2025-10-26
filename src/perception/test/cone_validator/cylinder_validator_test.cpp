#include <gtest/gtest.h>

#include <cone_validator/cylinder_validator.hpp>

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
};

/**
 * @brief Test case to validate if points are inside the small cylinder.
 */
TEST_F(CylinderValidatorTest, PointsInsideSmallCylinder) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  pcl::PointCloud<PointXYZIR> cloud;
  cloud.points.push_back(PointXYZIR{0.3, 0.3, 0.5, 0});
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  Cluster cylinderPointCloud(cloud_ptr);

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  // The centroid is at (0.3, 0.3, 0.5), so the distance to the point is 0.

  ASSERT_NEAR(result[0], 1.0, 1e-6);  // Distance XY ratio is 1 because the point is the centroid.
  ASSERT_NEAR(result[1], 1.0, 1e-6);  // Distance Z ratio is 1 because the point is the centroid.
  ASSERT_DOUBLE_EQ(result[2], 1.0);   // All points are inside the cylinder.
}

/**
 * @brief Test case to validate if points are inside the large cylinder.
 */
TEST_F(CylinderValidatorTest, PointsInsideLargeCylinder) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  pcl::PointCloud<PointXYZIR> cloud;
  cloud.points.push_back(PointXYZIR{0.6, 0.6, 1.2, 0});
  cloud.points.push_back(PointXYZIR{0.3, 0.3, 0.5, 0});
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  Cluster cylinderPointCloud(cloud_ptr);
  // Simulate height validator and set cluster as large.
  cylinderPointCloud.set_is_large();

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  // The centroid is at (0.45, 0.45, 0.85)
  // Both points are inside the large cylinder so the ratios are 1.

  ASSERT_NEAR(result[0], 1.0, 1e-6);  // Distance XY ratio is 1.
  ASSERT_NEAR(result[1], 1.0, 1e-6);  // Distance Z ratio is 1.
  ASSERT_DOUBLE_EQ(result[2], 1.0);   // All points are inside the cylinder.
}

/**
 * @brief Test case to validate if points are outside all cylinders.
 */
TEST_F(CylinderValidatorTest, PointsFarOutsideCylinders) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  pcl::PointCloud<PointXYZIR> cloud;
  cloud.points.push_back(PointXYZIR{1.0, 1.0, 2.0, 0});
  cloud.points.push_back(PointXYZIR{20.0, 20.0, 20.0, 0});
  cloud.points.push_back(PointXYZIR{0.3, 0.3, 0.5, 0});
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  Cluster cylinderPointCloud(cloud_ptr);

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  // The centroid is at (7.1, 7.1, 7.5)
  // All points are outside the cylinders, so the ratios are capped to 0.

  ASSERT_NEAR(result[0], 0.0, 1e-6);  // Distance XY ratio is 0.
  ASSERT_NEAR(result[1], 0.0, 1e-6);  // Distance Z ratio is 0.
  ASSERT_LT(result[2], 1.0);          // Some points are outside the cylinder.
}

/**
 * @brief Test case to validate if points are outside all cylinders.
 */
TEST_F(CylinderValidatorTest, PointsOutsideCylinders) {
  const CylinderValidator validator(0.5, 1.0, 0.7, 1.5, 0.5);

  pcl::PointCloud<PointXYZIR> cloud;
  cloud.points.push_back(PointXYZIR{1.0, 1.0, 2.0, 0});
  cloud.points.push_back(PointXYZIR{0.6, 0.6, 0.9, 0});
  cloud.points.push_back(PointXYZIR{0.3, 0.3, 0.5, 0});
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  Cluster cylinderPointCloud(cloud_ptr);

  std::vector<double> result = validator.coneValidator(&cylinderPointCloud, plane);

  // Centroid is at (0.633333, 0.633333, 1.13333)
  // Distance to (1,1,2) is ~1.04, so ratio is 0.5/1.04 ~ 0.48, so it is capped to 0.0

  ASSERT_EQ(result[0], 0.0);  // Distance XY ratio is capped to 0.0.
  ASSERT_GE(result[1], 0.5);  // Distance Z ratio is higher than cap.
  ASSERT_LE(result[1], 1.0);  // Distance Z ratio is capped at 1.
  ASSERT_LT(result[2], 1.0);  // Some points are outside the cylinder.
}
