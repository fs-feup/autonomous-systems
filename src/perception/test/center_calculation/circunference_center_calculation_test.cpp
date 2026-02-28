#include <gtest/gtest.h>

#include <center_calculation/circunference_center_calculation.hpp>
#include <cone_validator/cylinder_validator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <test_utils/pointcloud2_helper.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @brief Test class for setting up data and testing CircinferenceCenterCalculation algorithm.
 *
 */

class CircunferenceCenterCalculationTest : public ::testing::Test {
public:
  CircunferenceCenterCalculator center_calculator_;
  Plane plane_;

protected:
  void SetUp() override {
    center_calculator_ = CircunferenceCenterCalculator();
    plane_ = Plane(0, 0, 1, 0);
  }
};

/**
 * @brief Test case for finding the circunference of center (2, 0)
 *
 */
TEST_F(CircunferenceCenterCalculationTest, NormalUseCase) {
  std::vector<std::array<float, 5>> pts = {{0.0, 0.0, 1.0, 0.5, 39},
                                           {2.0, 2.0, 1.0, 1.0, 39},
                                           {4.0, 0.0, 1.0, 1.5, 39},
                                           {2.0, -2.0, 1.0, 1.5, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0, 1, 2, 3};
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  auto center = center_calculator_.calculate_center(input_cloud, indices, plane_);
  ASSERT_EQ(center.x(), 2);
  ASSERT_EQ(center.y(), 0);
}

/**
 * @brief Test case for finding the circunference of center (0, 3)
 *
 */
TEST_F(CircunferenceCenterCalculationTest, NormalUseCase2) {
  std::vector<std::array<float, 5>> pts = {{0.0, 0.0, 2.0, 0.5, 39},
                                           {3.0, 3.0, 2.0, 1.0, 39},
                                           {-3.0, 3.0, 2.0, 1.5, 39},
                                           {0.0, 6.0, 2.0, 1.5, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0, 1, 2, 3};
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  auto center = center_calculator_.calculate_center(input_cloud, indices, plane_);
  ASSERT_EQ(center.x(), 0);
  ASSERT_EQ(center.y(), 3);
}
