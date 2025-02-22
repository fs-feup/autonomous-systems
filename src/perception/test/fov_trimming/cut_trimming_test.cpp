#include "fov_trimming/cut_trimming.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Test class for setting up data and testing CutTrimming algorithm.
 *
 */
class CutTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    // All points rotated -90 degrees
    pcl_cloud->points.push_back(pcl::PointXYZI{1.1, -4.3, 0.5, 0.0});   // Inside
    pcl_cloud->points.push_back(pcl::PointXYZI{1.6, -29.0, 0.5, 0.0});  // Outside max range
    pcl_cloud->points.push_back(pcl::PointXYZI{1.0, -3.2, 3.0, 0.0});   // Above max height
    pcl_cloud->points.push_back(pcl::PointXYZI{0.1, -0.1, 0.5, 0.0});   // Below min range
    pcl_cloud->points.push_back(pcl::PointXYZI{-5.1, 2.0, 0.5, 0.0});   // Outside FOV trim angle

    pcl_cloud_empty.reset(new pcl::PointCloud<pcl::PointXYZI>);

    params.max_range = 1000.0;
    params.min_range = 0.0;
    params.max_height = 1000.0;
    params.lidar_height = 0.0;
    params.fov_trim_angle = 120.0;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_empty;
  TrimmingParameters params;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMaxRange) {
  params.max_range = 15.0;
  CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMaxHeight) {
  params.max_height = 2.0;
  CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 4);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMinRange) {
  params.min_range = 0.5;
  CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 4);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestFOVAngle) {
  params.fov_trim_angle = 60.0;
  CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 4);
}

/**
 * @brief Empty point cloud remains empty.
 *
 */
TEST_F(CutTrimmingTest, TestEmptyPointCloud) {
  CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(pcl_cloud_empty);

  ASSERT_EQ(pcl_cloud_empty->points.size(), 0);
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
  params.fov_trim_angle = 45.0;
  CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 1);
}