#include "fov_trimming/acceleration_trimming.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/trimming_parameters.hpp>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

/**
 * @brief Test class for setting up data and testing AccelerationTrimming algorithm.
 *
 */
class AccelerationTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    cloud.points.clear();
    // All points rotated -90 degrees.
    cloud.points.push_back(pcl::PointXYZI{1.1, -4.3, 0.5, 0.0});   // Inside
    cloud.points.push_back(pcl::PointXYZI{1.6, -29.0, 0.5, 0.0});  // Outside max range
    cloud.points.push_back(pcl::PointXYZI{1.0, -3.2, 3.0, 0.0});   // Above max height
    cloud.points.push_back(pcl::PointXYZI{0.1, -0.1, 0.5, 0.0});   // Below min range
    cloud.points.push_back(pcl::PointXYZI{-5.1, -5.0, 0.5, 0.0});  // Outside max y range

    // For the empty point cloud, just ensure it is empty.
    cloud_empty.points.clear();

    // Set trimming parameters.
    params.acc_max_range = 20.25;
    params.acc_fov_trim_angle = 90.0;
    params.min_range = 0.0;
    params.max_height = 10.0;
    params.lidar_height = 0;
    params.acc_max_y = 10.0;
    params.lidar_rotation = 90.0;
  }

  // Stack-allocated point clouds.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> cloud_empty;
  TrimmingParameters params;
};

/**
 * @brief Test case where points outside max range should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxRange) {
  // Wrap the stack object in a non-owning shared pointer.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxHeight) {
  params.max_height = 2.0;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMinRange) {
  params.min_range = 0.2;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 3);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(AccelerationTrimmingTest, TestMaxY) {
  params.acc_max_y = 4.0;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 3);
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
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_empty_ptr(
      &cloud_empty, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(cloud_empty_ptr);

  ASSERT_EQ(cloud_empty_ptr->points.size(), 0);
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
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const AccelerationTrimming acc_trimming(params);
  acc_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 1);
}
