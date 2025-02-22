#include "fov_trimming/acceleration_trimming.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/trimming_parameters.hpp>

/**
 * @brief Test class for setting up data and testing AccelerationTrimming algorithm.
 *
 */
class AccelerationTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    // All points rotated -90 degrees
    pcl_cloud->points.push_back(pcl::PointXYZI{1.1, -4.3, 0.5, 0.0});   // Inside
    pcl_cloud->points.push_back(pcl::PointXYZI{1.6, -29.0, 0.5, 0.0});  // Outside max range
    pcl_cloud->points.push_back(pcl::PointXYZI{1.0, -3.2, 3.0, 0.0});   // Above max height
    pcl_cloud->points.push_back(pcl::PointXYZI{0.1, -0.1, 0.5, 0.0});   // Below min range
    pcl_cloud->points.push_back(pcl::PointXYZI{-5.1, -5.0, 0.5, 0.0});  // Outside max y range

    pcl_cloud_empty.reset(new pcl::PointCloud<pcl::PointXYZI>);

    params.min_range = 0.0;
    params.max_height = 10.0;
    params.lidar_height = 0;
    params.acc_max_y = 10.0;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_empty;
  TrimmingParameters params;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxRange) {
  AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxHeight) {
  params.max_height = 2.0;
  AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMinRange) {
  params.min_range = 0.2;
  AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxY) {
  params.acc_max_y = 4.0;
  AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 3);
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
  AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(pcl_cloud_empty);

  ASSERT_EQ(pcl_cloud_empty->points.size(), 0);
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
  AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 1);
}