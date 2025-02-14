#include "fov_trimming/skidpad_trimming.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SkidpadTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl_cloud->points.push_back(pcl::PointXYZI{1.1, -4.3, 0.5, 0.0});   // Inside
    pcl_cloud->points.push_back(pcl::PointXYZI{1.6, -39.0, 0.5, 0.0});  // Outside max range
    pcl_cloud->points.push_back(pcl::PointXYZI{1.0, -3.2, 3.0, 0.0});   // Above max height
    pcl_cloud->points.push_back(pcl::PointXYZI{0.1, -0.1, 0.5, 0.0});   // Below min range
    pcl_cloud->points.push_back(pcl::PointXYZI{2.5, -2.0, 0.5, 0.0});   // Outside FOV trim angle

    pcl_cloud_empty.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_empty;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMaxRange) {
  SkidpadTrimming skidpad_trimming(0.0, 1000.0, 1.5);
  skidpad_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 4);
}

/**
 * @brief Test case where points above max height and max range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMaxHeight) {
  SkidpadTrimming skidpad_trimming(0.0, 2.5, 1.5);
  skidpad_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

/**
 * @brief Test case where points within min range and above max range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMinRange) {
  SkidpadTrimming skidpad_trimming(0.3, 1000.0, 1.5);
  skidpad_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

/**
 * @brief Test case where points outside FOV trim angle and above max range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestFOVAngle) {
  SkidpadTrimming skidpad_trimming(0.0, 1000.0, 2.0);
  skidpad_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

/**
 * @brief Empty point cloud remains empty.
 *
 */
TEST_F(SkidpadTrimmingTest, TestEmptyPointCloud) {
  SkidpadTrimming skidpad_trimming(0.0, 1000.0, 1.5);
  skidpad_trimming.fov_trimming(pcl_cloud_empty);

  ASSERT_EQ(pcl_cloud_empty->points.size(), 0);
}

/**
 * @brief Test for the general result of skidpad_trimming.
 *
 */
TEST_F(SkidpadTrimmingTest, TestGeneralResult) {
  SkidpadTrimming skidpad_trimming(0.4, 2.5, 4.0);
  skidpad_trimming.fov_trimming(pcl_cloud);

  ASSERT_EQ(pcl_cloud->points.size(), 1);
}
