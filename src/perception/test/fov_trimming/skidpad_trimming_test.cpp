#include "fov_trimming/skidpad_trimming.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SkidpadTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl_cloud->points.push_back(pcl::PointXYZI{1.0, -4.0, 0.5, 0.0});    // Inside
    pcl_cloud->points.push_back(pcl::PointXYZI{1.0, -21.0, 0.5, 0.0});   // Outside max range
    pcl_cloud->points.push_back(pcl::PointXYZI{1.0, -3.5, 3.0, 0.0});    // Above max height
    pcl_cloud->points.push_back(pcl::PointXYZI{0.05, -0.05, 0.5, 0.0});  // Below min range
    pcl_cloud->points.push_back(pcl::PointXYZI{2.0, -0.2, 0.5, 0.0});    // Outside FOV trim angle

    pcl_cloud_empty.reset(new pcl::PointCloud<pcl::PointXYZI>);

    params.min_range = 0.0;
    params.max_height = 1000.0;
    params.lidar_height = 0.0;
    params.min_distance_to_cone = 1.5;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_empty;
  TrimmingParameters params;
};

TEST_F(SkidpadTrimmingTest, TestMaxRange) {
  SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(pcl_cloud);
  ASSERT_EQ(pcl_cloud->points.size(), 4);
}

TEST_F(SkidpadTrimmingTest, TestMaxHeight) {
  params.max_height = 2.5;
  SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(pcl_cloud);
  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

TEST_F(SkidpadTrimmingTest, TestMinRange) {
  params.min_range = 0.3;
  SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(pcl_cloud);
  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

TEST_F(SkidpadTrimmingTest, TestFOVAngle) {
  params.min_distance_to_cone = 1.7;
  params.fov_trim_angle =
      90 - std::acos(1.5 / std::max(params.min_distance_to_cone, 1.5)) * 180 / M_PI;
  SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(pcl_cloud);
  ASSERT_EQ(pcl_cloud->points.size(), 3);
}

TEST_F(SkidpadTrimmingTest, TestEmptyPointCloud) {
  SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(pcl_cloud_empty);
  ASSERT_EQ(pcl_cloud_empty->points.size(), 0);
}

TEST_F(SkidpadTrimmingTest, TestGeneralResult) {
  params.min_range = 0.4;
  params.max_height = 2.5;
  params.min_distance_to_cone = 4.0;
  params.fov_trim_angle =
      90 - std::acos(1.5 / std::max(params.min_distance_to_cone, 1.5)) * 180 / M_PI;
  SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(pcl_cloud);
  ASSERT_EQ(pcl_cloud->points.size(), 1);
}
