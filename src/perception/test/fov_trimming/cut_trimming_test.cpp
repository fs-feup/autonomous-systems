#include "fov_trimming/cut_trimming.hpp"

#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <test_utils/pointcloud2_helper.hpp>

/**
 * @brief Test class for setting up data and testing CutTrimming algorithm.
 *
 */
class CutTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    std::vector<std::array<float, 5>> pts = {
        {1.1, -4.3, 0.5, 0.0, 39},   // Inside
        {1.6, -29.0, 0.5, 0.0, 39},  // Outside max range
        {1.0, -3.2, 3.0, 0.0, 39},   // Above max height
        {0.1, -0.1, 0.5, 0.0, 39},   // Below min range
        {-5.1, 2.0, 0.5, 0.0, 39}    // Outside FOV trim angle
    };
    cloud = test_utils::make_lidar_pointcloud2(pts);
    cloud_empty = test_utils::make_lidar_pointcloud2({});

    params.max_range = 1000.0;
    params.min_range = 0.0;
    params.max_height = 1000.0;
    params.lidar_height = 0.0;
    params.apply_fov_trimming = false;
    params.fov = 360.0;
    params.apply_rotation = true;
    params.rotation = 90.0;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_empty;
  TrimmingParameters params;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMaxRange) {
  params.max_range = 15.0;
  auto input_cloud = cloud;
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMaxHeight) {
  params.max_height = 2.0;
  auto input_cloud = cloud;
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 4);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMinRange) {
  params.min_range = 0.5;
  auto input_cloud = cloud;
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 4);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestFOVAngle) {
  params.apply_fov_trimming = true;
  params.fov = 180.0;
  auto input_cloud = cloud;
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 4);
}

/**
 * @brief Empty point cloud remains empty.
 *
 */
TEST_F(CutTrimmingTest, TestEmptyPointCloud) {
  auto input_cloud = cloud_empty;
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 0);
}

/**
 * @brief Test for the general result of cut_trimming.
 *
 */
TEST_F(CutTrimmingTest, TestGeneralResult) {
  params.max_range = 10.0;
  params.min_range = 0.2;
  params.max_height = 2.5;
  params.lidar_height = 0.0;
  params.apply_fov_trimming = true;
  params.fov = 45.0;
  auto input_cloud = cloud;
  auto output_cloud = test_utils::make_lidar_pointcloud2({});
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(input_cloud, output_cloud);
  ASSERT_EQ(output_cloud->width, 1);
}
