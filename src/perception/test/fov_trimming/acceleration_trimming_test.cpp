#include "fov_trimming/acceleration_trimming.hpp"

#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <test_utils/pointcloud2_helper.hpp>
#include <utils/trimming_parameters.hpp>

/**
 * @brief Test class for setting up data and testing AccelerationTrimming algorithm.
 *
 */
class AccelerationTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    params.acc_max_range = 20.25;
    params.apply_fov_trimming = false;
    params.fov = 180.0;
    params.min_range = 0.0;
    params.max_height = 10.0;
    params.lidar_height = 0;
    params.acc_max_y = 10.0;
    params.apply_rotation = true;
    params.rotation = 90.0;
  }

  TrimmingParameters params;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxRange) {
  std::vector<std::array<float, 5>> pts = {{1.1, -4.3, 0.5, 0.0, 39},
                                           {1.6, -29.0, 0.5, 0.0, 39},
                                           {1.0, -3.2, 3.0, 0.0, 39},
                                           {0.1, -0.1, 0.5, 0.0, 39},
                                           {-5.1, -5.0, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxHeight) {
  params.max_height = 2.0;
  std::vector<std::array<float, 5>> pts = {{1.1, -4.3, 0.5, 0.0, 39},
                                           {1.6, -29.0, 0.5, 0.0, 39},
                                           {1.0, -3.2, 3.0, 0.0, 39},
                                           {0.1, -0.1, 0.5, 0.0, 39},
                                           {-5.1, -5.0, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 3);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMinRange) {
  params.min_range = 0.2;
  std::vector<std::array<float, 5>> pts = {{1.1, -4.3, 0.5, 0.0, 39},
                                           {1.6, -29.0, 0.5, 0.0, 39},
                                           {1.0, -3.2, 3.0, 0.0, 39},
                                           {0.1, -0.1, 0.5, 0.0, 39},
                                           {-5.1, -5.0, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 3);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxY) {
  params.acc_max_y = 4.0;
  std::vector<std::array<float, 5>> pts = {{1.1, -4.3, 0.5, 0.0, 39},
                                           {1.6, -29.0, 0.5, 0.0, 39},
                                           {1.0, -3.2, 3.0, 0.0, 39},
                                           {0.1, -0.1, 0.5, 0.0, 39},
                                           {-5.1, -5.0, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 3);
}

/**
 * @brief Empty point cloud remains empty.
 *
 */
TEST_F(AccelerationTrimmingTest, TestEmptyPointCloud) {
  params.min_range = 0.2;
  params.max_height = 2.0;
  params.lidar_height = 0;
  params.acc_max_y = 4.0;
  auto input_cloud = test_utils::make_lidar_pointcloud2({});
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 0);
}

/**
 * @brief Test case for the general result of acc_trimming.
 *
 */
TEST_F(AccelerationTrimmingTest, TestGeneralResult) {
  params.min_range = 0.2;
  params.max_height = 2.0;
  params.lidar_height = 0;
  params.acc_max_y = 4.0;
  std::vector<std::array<float, 5>> pts = {{1.1, -4.3, 0.5, 0.0, 39},
                                           {1.6, -29.0, 0.5, 0.0, 39},
                                           {1.0, -3.2, 3.0, 0.0, 39},
                                           {0.1, -0.1, 0.5, 0.0, 39},
                                           {-5.1, -5.0, 0.5, 0.0, 39}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 1);
}
