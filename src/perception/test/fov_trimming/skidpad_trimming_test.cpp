#include "fov_trimming/skidpad_trimming.hpp"

#include <gtest/gtest.h>

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
    cloud.points.push_back(PointXYZIR{1.0, -4.0, 0.5, 0.0});    // Inside
    cloud.points.push_back(PointXYZIR{1.0, -21.0, 0.5, 0.0});   // Outside max range
    cloud.points.push_back(PointXYZIR{1.0, -3.5, 3.0, 0.0});    // Above max height
    cloud.points.push_back(PointXYZIR{0.05, -0.05, 0.5, 0.0});  // Below min range
    cloud.points.push_back(PointXYZIR{2.0, -0.2, 0.5, 0.0});    // Outside FOV trim angle

    // Ensure the empty cloud is empty.
    cloud_empty.points.clear();

    // Set initial trimming parameters.
    params.skid_max_range = 20.25;
    params.min_range = 0.0;
    params.max_height = 1000.0;
    params.lidar_height = 0.0;
    params.skid_fov_trim_angle = 90.0;
    params.lidar_rotation = 90.0;
    // fov_trim_angle is not set here for every test, but will be adjusted in tests as needed.
  }

  // Stack-allocated point clouds.
  pcl::PointCloud<PointXYZIR> cloud;
  pcl::PointCloud<PointXYZIR> cloud_empty;
  TrimmingParameters params;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMaxRange) {
  // Wrap the stack cloud with a non-owning shared pointer.
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMaxHeight) {
  params.max_height = 2.5;
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestMinRange) {
  params.min_range = 0.3;
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(SkidpadTrimmingTest, TestFOVAngle) {
  params.skid_fov_trim_angle = 75.0;
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

/**
 * @brief Empty point cloud remains empty.
 *
 */
TEST_F(SkidpadTrimmingTest, TestEmptyPointCloud) {
  const pcl::PointCloud<PointXYZIR>::Ptr empty_ptr(&cloud_empty,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(empty_ptr);
  ASSERT_EQ(empty_ptr->points.size(), 0);
}

/**
 * @brief Test for the general result of skidpad_trimming.
 *
 */
TEST_F(SkidpadTrimmingTest, TestGeneralResult) {
  params.min_range = 0.4;
  params.max_height = 2.5;
  params.skid_fov_trim_angle = 35.0;
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const SkidpadTrimming skidpad_trimming(params);
  skidpad_trimming.fov_trimming(cloud_ptr);
  ASSERT_EQ(cloud_ptr->points.size(), 1);
}
