#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <center_calculation/circunferece_center_calculation.hpp>
#include <cone_validator/cylinder_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test class for setting up data and testing CircinferenceCenterCalculation algorithm.
 *
 */
class CircunferenceCenterCalculationTest : public ::testing::Test {
public:
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
      pcl_cloud;  ///< Point Cloud representing the circumference (x - 2)^2 + y^2 = 4
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
      pcl_cloud2;  ///< Point Cloud representing the circumference x^2 + (y-3)^2 = 9
  CircunferenceCenterCalculation center_calculator_;  ///< Center Calculator
  Plane plane_;                                       ///< Plane z = 0

protected:
  /**
   * @brief Set up the test environment before each test case.
   */
  void SetUp() override {
    center_calculator_ = CircunferenceCenterCalculation();

    pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl_cloud->points.push_back(pcl::PointXYZI{0.0, 0.0, 1.0, 0.5});
    pcl_cloud->points.push_back(pcl::PointXYZI{2.0, 2.0, 1.0, 1.0});
    pcl_cloud->points.push_back(pcl::PointXYZI{4.0, 0.0, 1.0, 1.5});
    pcl_cloud->points.push_back(pcl::PointXYZI{2.0, -2.0, 1.0, 1.5});

    pcl_cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl_cloud2->points.push_back(pcl::PointXYZI{0.0, 0.0, 2.0, 0.5});
    pcl_cloud2->points.push_back(pcl::PointXYZI{3.0, 3.0, 2.0, 1.0});
    pcl_cloud2->points.push_back(pcl::PointXYZI{-3.0, 3.0, 2.0, 1.5});
    pcl_cloud2->points.push_back(pcl::PointXYZI{0.0, 6.0, 2.0, 1.5});

    plane_ = Plane(0, 0, 1, 0);  // Plane z = 0
  }
};

/**
 * @brief Test case for finding the circunference of center (2, 0)
 *
 */
TEST_F(CircunferenceCenterCalculationTest, NormalUseCase) {
  auto center = center_calculator_.calculate_center(pcl_cloud, plane_);

  ASSERT_EQ(center.x(), 2);
  ASSERT_EQ(center.y(), 0);
}

/**
 * @brief Test case for finding the circunference of center (0, 3)
 *
 */
TEST_F(CircunferenceCenterCalculationTest, NormalUseCase2) {
  auto center = center_calculator_.calculate_center(pcl_cloud2, plane_);

  ASSERT_EQ(center.x(), 0);
  ASSERT_EQ(center.y(), 3);
}
