#include "fov_trimming/cut_trimming.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

/**
 * @brief Test class for setting up data and testing CutTrimming algorithm.
 *
 */
class CutTrimmingTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Instead of using dynamic memory allocation,
    // create the point clouds on the stack.
    cloud.points.clear();
    // All points rotated -90 degrees
    cloud.points.push_back(pcl::PointXYZI{1.1, -4.3, 0.5, 0.0});   // Inside
    cloud.points.push_back(pcl::PointXYZI{1.6, -29.0, 0.5, 0.0});  // Outside max range
    cloud.points.push_back(pcl::PointXYZI{1.0, -3.2, 3.0, 0.0});   // Above max height
    cloud.points.push_back(pcl::PointXYZI{0.1, -0.1, 0.5, 0.0});   // Below min range
    cloud.points.push_back(pcl::PointXYZI{-5.1, 2.0, 0.5, 0.0});   // Outside FOV trim angle

    // Ensure the empty cloud is empty.
    cloud_empty.points.clear();

    // Set initial trimming parameters.
    params.max_range = 1000.0;
    params.min_range = 0.0;
    params.max_height = 1000.0;
    params.lidar_height = 0.0;
    params.fov_trim_angle = 120.0;
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
TEST_F(CutTrimmingTest, TestMaxRange) {
  params.max_range = 15.0;
  // Wrap the stack object in a non-owning shared pointer.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 4);
}

/**
 * @brief Test case where points above max height should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMaxHeight) {
  params.max_height = 2.0;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 4);
}

/**
 * @brief Test case where points within min range should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestMinRange) {
  params.min_range = 0.5;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 4);
}

/**
 * @brief Test case where points outside FOV trim angle should be removed.
 *
 */
TEST_F(CutTrimmingTest, TestFOVAngle) {
  params.fov_trim_angle = 60.0;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 4);
}

/**
 * @brief Empty point cloud remains empty.
 *
 */
TEST_F(CutTrimmingTest, TestEmptyPointCloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &cloud_empty, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(empty_ptr);

  ASSERT_EQ(empty_ptr->points.size(), 0);
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
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const CutTrimming cut_trimming(params);
  cut_trimming.fov_trimming(cloud_ptr);

  ASSERT_EQ(cloud_ptr->points.size(), 1);
}
