#include "ground_removal/grid_ransac.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <memory>

// Non-owning deleter: does nothing.
template <typename T>
struct non_owning_deleter {
  void operator()(T*) const {}
};

class GridRANSACTest : public ::testing::Test {
public:
  void SetUp() override {
    // Create the point clouds on the stack.
    cloud.points.clear();
    cloud.points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 0.5});
    cloud.points.push_back(pcl::PointXYZI{0.0, 1.0, 0.0, 1.0});
    cloud.points.push_back(pcl::PointXYZI{0.0, 0.0, 1.0, 1.5});
    cloud.points.push_back(pcl::PointXYZI{0.0060, 0.0060, 0.0060, 2.0});
    cloud.points.push_back(pcl::PointXYZI{10, 10, 10, 2.5});

    empty_cloud.points.clear();

    cloud_3_points.points.clear();
    cloud_3_points.points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 0.5});
    cloud_3_points.points.push_back(pcl::PointXYZI{0.0, 1.0, 0.0, 1.0});
    cloud_3_points.points.push_back(pcl::PointXYZI{0.0, 0.0, 1.0, 1.5});

    cloud_two_grids.points.clear();
    cloud_two_grids.points.push_back(pcl::PointXYZI{55.0, 55.0, 100.0, 0.5});
    cloud_two_grids.points.push_back(pcl::PointXYZI{60.0, 60.0, 100.0, 1.0});
    cloud_two_grids.points.push_back(pcl::PointXYZI{55.0, 55.0, 100.0, 1.5});
    cloud_two_grids.points.push_back(pcl::PointXYZI{-55.0, -55.0, 0.0, 0.5});
    cloud_two_grids.points.push_back(pcl::PointXYZI{-50.0, -50.0, 0.0, 1.0});
    cloud_two_grids.points.push_back(pcl::PointXYZI{-60.0, -60.0, 0.0, 1.5});

    cloud_four_grids.points.clear();
    // first quadrant
    cloud_four_grids.points.push_back(pcl::PointXYZI{55.0, 55.0, 100.0, 0.5});
    cloud_four_grids.points.push_back(pcl::PointXYZI{60.0, 60.0, 100.0, 1.0});
    cloud_four_grids.points.push_back(pcl::PointXYZI{55.0, 55.0, 3.0, 1.5});
    // second quadrant
    cloud_four_grids.points.push_back(pcl::PointXYZI{55.0, -55.0, 0.0, 0.5});
    cloud_four_grids.points.push_back(pcl::PointXYZI{60.0, -60.0, 0.0, 1.0});
    cloud_four_grids.points.push_back(pcl::PointXYZI{55.0, -55.0, 0.0, 1.5});
    // third quadrant
    cloud_four_grids.points.push_back(pcl::PointXYZI{-55.0, -55.0, -130.0, 0.5});
    cloud_four_grids.points.push_back(pcl::PointXYZI{-50.0, -50.0, -130.0, 1.0});
    cloud_four_grids.points.push_back(pcl::PointXYZI{-60.0, -60.0, -130.0, 1.5});
    // fourth quadrant
    cloud_four_grids.points.push_back(pcl::PointXYZI{55.0, -55.0, 400.0, 0.5});
    cloud_four_grids.points.push_back(pcl::PointXYZI{50.0, -50.0, 400.0, 1.0});
    cloud_four_grids.points.push_back(pcl::PointXYZI{60.0, -60.0, 400.0, 1.5});

    two_radius.points.clear();
    two_radius.points.push_back(pcl::PointXYZI{7.0, 7.0, 400.0, 0.5});
    two_radius.points.push_back(pcl::PointXYZI{8.0, 4.0, 400.0, 1.0});
    two_radius.points.push_back(pcl::PointXYZI{9.0, 9.0, 400.0, 1.5});
    two_radius.points.push_back(pcl::PointXYZI{10.0, 10.0, -400.0, 0.5});
    two_radius.points.push_back(pcl::PointXYZI{11.0, 11.0, -400.0, 1.0});
    two_radius.points.push_back(pcl::PointXYZI{12.0, 13.0, -400.0, 1.5});
  }

  // Stack-allocated point clouds.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  pcl::PointCloud<pcl::PointXYZI> cloud_3_points;
  pcl::PointCloud<pcl::PointXYZI> cloud_two_grids;
  pcl::PointCloud<pcl::PointXYZI> cloud_four_grids;
  pcl::PointCloud<pcl::PointXYZI> two_radius;
};

//
// For each test, we wrap the stack cloud in a non-owning shared pointer.
// The following tests use the same test scenarios as before.
//

TEST_F(GridRANSACTest, TestBigEpsilon) {
  auto ground_removal = std::make_shared<GridRANSAC>(10000, 1, 1, 100);
  Plane plane;

  // Create a shared pointer wrapping the stack cloud.
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud, plane);

  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestCommonScenario) {
  auto ground_removal = std::make_shared<GridRANSAC>(0.05, 100, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

TEST_F(GridRANSACTest, TestCommonScenario2) {
  auto ground_removal = std::make_shared<GridRANSAC>(0.5, 100, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 1);
}

TEST_F(GridRANSACTest, TestThresholdZero) {
  auto ground_removal = std::make_shared<GridRANSAC>(0, 10, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 5);
}

TEST_F(GridRANSACTest, TestZeroRepetitions) {
  auto ground_removal = std::make_shared<GridRANSAC>(100, 0, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestSmallEpsilon) {
  auto ground_removal = std::make_shared<GridRANSAC>(0.00000000000000001, 10, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 2);
}

TEST_F(GridRANSACTest, TestBigEpsilon2) {
  auto ground_removal = std::make_shared<GridRANSAC>(1000000, 1, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestCommonScenario3Points) {
  auto ground_removal = std::make_shared<GridRANSAC>(100, 100, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3_ptr(
      &cloud_3_points, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud3_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, Test3PointsThresholdZero) {
  auto ground_removal = std::make_shared<GridRANSAC>(0, 100, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3_ptr(
      &cloud_3_points, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(cloud3_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 3);
}

TEST_F(GridRANSACTest, TestEmptyPointCloud) {
  auto ground_removal = std::make_shared<GridRANSAC>(100, 100, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &empty_cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(empty_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestEmptyPointCloud2) {
  auto ground_removal = std::make_shared<GridRANSAC>(0, 0, 1, 100);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &empty_cloud, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(empty_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoGrids) {
  auto ground_removal = std::make_shared<GridRANSAC>(100, 1000, 2, 10000);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr two_grids_ptr(
      &cloud_two_grids, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(two_grids_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGrids) {
  auto ground_removal = std::make_shared<GridRANSAC>(100, 10000, 4, 10000);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr four_grids_ptr(
      &cloud_four_grids, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(four_grids_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGridsTwoPlanes) {
  auto ground_removal = std::make_shared<GridRANSAC>(0.000001, 10000, 2, 10000);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr four_grids_ptr(
      &cloud_four_grids, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(four_grids_ptr, ground_removed_cloud, plane);
  ASSERT_NE(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGridsSinglePlane) {
  auto ground_removal = std::make_shared<GridRANSAC>(0.000001, 10000, 1, 10000);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr four_grids_ptr(
      &cloud_four_grids, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(four_grids_ptr, ground_removed_cloud, plane);
  ASSERT_NE(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoRadius) {
  auto ground_removal = std::make_shared<GridRANSAC>(0.000001, 10000, 1, 13);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr radius_ptr(
      &two_radius, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(radius_ptr, ground_removed_cloud, plane);
  ASSERT_EQ(ground_removed_cloud->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoRadiusOnePlane) {
  auto ground_removal = std::make_shared<GridRANSAC>(0.000001, 10000, 1, 10000);
  Plane plane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr radius_ptr(
      &two_radius, non_owning_deleter<pcl::PointCloud<pcl::PointXYZI>>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ground_removal->ground_removal(radius_ptr, ground_removed_cloud, plane);
  ASSERT_NE(ground_removed_cloud->points.size(), 0);
}
