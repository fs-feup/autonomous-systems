#include "ground_removal/grid_ransac.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <memory>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
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

    split_params.fov_angle = 360.0;
    split_params.n_angular_grids = 1;
    split_params.radius_resolution = 100.0;
  }

  // Stack-allocated point clouds.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  pcl::PointCloud<pcl::PointXYZI> cloud_3_points;
  pcl::PointCloud<pcl::PointXYZI> cloud_two_grids;
  pcl::PointCloud<pcl::PointXYZI> cloud_four_grids;
  pcl::PointCloud<pcl::PointXYZI> two_radius;
  pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;

  SplitParameters split_params;
};

//
// For each test, we wrap the stack cloud in a non-owning shared pointer.
// The following tests use the same test scenarios as before.
//

TEST_F(GridRANSACTest, TestBigEpsilon) {
  const auto ground_removal = std::make_shared<GridRANSAC>(10'000, 1);
  Plane plane;

  // Create a shared pointer wrapping the stack cloud.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, plane, split_params);

  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestCommonScenario) {
  const auto ground_removal = std::make_shared<GridRANSAC>(0.05, 100);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 2);
}

TEST_F(GridRANSACTest, TestCommonScenario2) {
  const auto ground_removal = std::make_shared<GridRANSAC>(0.5, 100);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 1);
}

TEST_F(GridRANSACTest, TestThresholdZero) {
  const auto ground_removal = std::make_shared<GridRANSAC>(0, 10);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 5);
}

TEST_F(GridRANSACTest, TestZeroRepetitions) {
  const auto ground_removal = std::make_shared<GridRANSAC>(100, 0);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestSmallEpsilon) {
  const auto ground_removal = std::make_shared<GridRANSAC>(0.000'000'000'000'000'01, 10);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 2);
}

TEST_F(GridRANSACTest, TestBigEpsilon2) {
  const auto ground_removal = std::make_shared<GridRANSAC>(1'000'000, 1);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestCommonScenario3Points) {
  const auto ground_removal = std::make_shared<GridRANSAC>(100, 100);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3_ptr(
      &cloud_3_points, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud3_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, Test3PointsThresholdZero) {
  const auto ground_removal = std::make_shared<GridRANSAC>(0, 100);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3_ptr(
      &cloud_3_points, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(cloud3_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 3);
}

TEST_F(GridRANSACTest, TestEmptyPointCloud) {
  const auto ground_removal = std::make_shared<GridRANSAC>(100, 100);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &empty_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(empty_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestEmptyPointCloud2) {
  const auto ground_removal = std::make_shared<GridRANSAC>(0, 0);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &empty_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(empty_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoGrids) {
  split_params.n_angular_grids = 2;
  split_params.radius_resolution = 10'000;
  const auto ground_removal = std::make_shared<GridRANSAC>(100, 1000);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr two_grids_ptr(
      &cloud_two_grids, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(two_grids_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGrids) {
  split_params.n_angular_grids = 4;
  split_params.radius_resolution = 10'000;
  const auto ground_removal = std::make_shared<GridRANSAC>(100, 10'000);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr four_grids_ptr(
      &cloud_four_grids, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(four_grids_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGridsTwoPlanes) {
  split_params.n_angular_grids = 2;
  split_params.radius_resolution = 10'000;
  const auto ground_removal = std::make_shared<GridRANSAC>(0.000'001, 10'000);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr four_grids_ptr(
      &cloud_four_grids, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(four_grids_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_NE(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestFourGridsSinglePlane) {
  split_params.radius_resolution = 10'000;
  const auto ground_removal = std::make_shared<GridRANSAC>(0.000'001, 10'000);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr four_grids_ptr(
      &cloud_four_grids, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(four_grids_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_NE(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoRadius) {
  split_params.n_angular_grids = 1;
  split_params.radius_resolution = 13;
  const auto ground_removal = std::make_shared<GridRANSAC>(0.000'001, 10'000);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr radius_ptr(
      &two_radius, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(radius_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}

TEST_F(GridRANSACTest, TestTwoRadiusOnePlane) {
  split_params.radius_resolution = 10'000;
  const auto ground_removal = std::make_shared<GridRANSAC>(0.000'001, 10'000);
  Plane plane;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr radius_ptr(
      &two_radius, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  ground_removal->ground_removal(radius_ptr, ground_removed_cloud_ptr, plane, split_params);
  ASSERT_NE(ground_removed_cloud_ptr->points.size(), 0);
}
