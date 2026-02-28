#include "fov_trimming/skidpad_trimming.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <test_utils/pointcloud2_helper.hpp>

class SkidpadTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    params.skid_max_range = 20.25;
    params.min_range = 0.0;
    params.max_height = 1000.0;
    params.lidar_height = 0.0;
    params.apply_fov_trimming = false;
    params.fov = 180.0;
    params.apply_rotation = true;
    params.rotation = 90.0;
  }

  TrimmingParameters params;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMaxRange) {
  std::vector<std::array<float, 5>> pts = {
      {1.0, -4.0, 0.5, 0.0, 39},    // Inside
      {1.0, -21.0, 0.5, 0.0, 39},   // Outside max range
      {1.0, -3.5, 3.0, 0.0, 39},    // Above max height
      {0.05, -0.05, 0.5, 0.0, 39},  // Below min range
      {2.0, -0.2, 0.5, 0.0, 39}     // Outside FOV trim angle
  };
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMaxHeight) {
  params.max_height = 2.5;
  std::vector<std::array<float, 5>> pts = {{1.0, -4.0, 0.5, 0.0, 39},
                                           {1.0, -21.0, 0.5, 0.0, 39},
                                           {1.0, -3.5, 3.0, 0.0, 39},
                                           {0.05, -0.05, 0.5, 0.0, 39},
                                           {2.0, -0.2, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 3);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMinRange) {
  params.min_range = 0.3;
  std::vector<std::array<float, 5>> pts = {{1.0, -4.0, 0.5, 0.0, 39},
                                           {1.0, -21.0, 0.5, 0.0, 39},
                                           {1.0, -3.5, 3.0, 0.0, 39},
                                           {0.05, -0.05, 0.5, 0.0, 39},
                                           {2.0, -0.2, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 3);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestFOVAngle) {
  params.apply_fov_trimming = true;
  params.fov = 180.0;
  params.max_range = 50.0;
  std::vector<std::array<float, 5>> pts = {{1.0, -4.0, 0.5, 0.0, 39},
                                           {1.0, -21.0, 0.5, 0.0, 39},
                                           {1.0, -3.5, 3.0, 0.0, 39},
                                           {0.05, -0.05, 0.5, 0.0, 39},
                                           {2.0, -0.2, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 4);
}

/**
 * @brief Empty point cloud remains empty.
 *
 */
TEST_F(SkidpadTrimmingTest, TestEmptyPointCloud) {
  auto input_cloud = test_utils::make_lidar_pointcloud2({});
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 0);
}

/**
 * @brief Test for the general result of skidpad_trimming.
 *
 */
TEST_F(SkidpadTrimmingTest, TestGeneralResult) {
  params.min_range = 0.4;
  params.max_height = 2.5;
  params.apply_fov_trimming = true;
  params.fov = 35.0;
  std::vector<std::array<float, 5>> pts = {{1.0, -4.0, 0.5, 0.0, 39},
                                           {1.0, -21.0, 0.5, 0.0, 39},
                                           {1.0, -3.5, 3.0, 0.0, 39},
                                           {0.05, -0.05, 0.5, 0.0, 39},
                                           {2.0, -0.2, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 1);
}
