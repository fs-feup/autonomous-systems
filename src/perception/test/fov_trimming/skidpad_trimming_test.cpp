#include "fov_trimming/skidpad_trimming.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

class SkidpadTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create the point cloud on the stack.
    cloud.points.clear();
    cloud.points.push_back(pcl::PointXYZI{1.0, -4.0, 0.5, 0.0});    // Inside
    cloud.points.push_back(pcl::PointXYZI{1.0, -21.0, 0.5, 0.0});   // Outside max range
    cloud.points.push_back(pcl::PointXYZI{1.0, -3.5, 3.0, 0.0});    // Above max height
    cloud.points.push_back(pcl::PointXYZI{0.05, -0.05, 0.5, 0.0});  // Below min range
    cloud.points.push_back(pcl::PointXYZI{2.0, -0.2, 0.5, 0.0});    // Outside FOV trim angle

    // Ensure the empty cloud is empty.
    cloud_empty.points.clear();

    // Set initial trimming parameters.
    params.min_range = 0.0;
    params.max_height = 1000.0;
    params.lidar_height = 0.0;
    params.min_distance_to_cone = 1.5;
    // fov_trim_angle is not set here for every test, but will be adjusted in tests as needed.
  }

  // Stack-allocated point clouds.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> cloud_empty;
  TrimmingParameters params;
};

TEST_F(SkidpadTrimmingTest, TestMaxRange) {
  // Wrap the stack cloud with a non-owning shared pointer.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 4);
}

TEST_F(SkidpadTrimmingTest, TestMaxHeight) {
  params.max_height = 2.5;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

TEST_F(SkidpadTrimmingTest, TestMinRange) {
  params.min_range = 0.3;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

TEST_F(SkidpadTrimmingTest, TestFOVAngle) {
  params.min_distance_to_cone = 1.7;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

TEST_F(SkidpadTrimmingTest, TestEmptyPointCloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &cloud_empty, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(empty_ptr);
  ASSERT_EQ(empty_ptr->points.size(), 0);
}

TEST_F(SkidpadTrimmingTest, TestGeneralResult) {
  params.min_range = 0.4;
  params.max_height = 2.5;
  params.min_distance_to_cone = 4.0;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 1);
}
