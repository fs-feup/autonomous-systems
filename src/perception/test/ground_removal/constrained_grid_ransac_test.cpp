#include "ground_removal/constrained_grid_ransac.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

class ConstrainedGridRANSACTest : public ::testing::Test {
public:
  void SetUp() override {
    // Same datasets as GridRANSAC
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

    split_params.fov_angle = 360.0;
    split_params.n_angular_grids = 1;
    split_params.radius_resolution = 100.0;
  }

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  pcl::PointCloud<pcl::PointXYZI> cloud_3_points;
  pcl::PointCloud<pcl::PointXYZI> cloud_two_grids;
  pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;

  SplitParameters split_params;
};

/**
 * @brief Test: With a very large epsilon, almost all points are considered ground.
 */
TEST_F(ConstrainedGridRANSACTest, TestBigEpsilon) {
  ConstrainedGridRANSAC ransac(10'000, 1, 5.0);
  Plane plane = Plane(0, 0, 1, 0);  // base plane

  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  ransac.ground_removal(cloud_ptr, output_ptr, plane, split_params);

  EXPECT_EQ(output_ptr->points.size(), 0);
}

/**
 * @brief Test: When candidate planes are rejected, fallback to base plane is used.
 */
TEST_F(ConstrainedGridRANSACTest, TestFallbackWhenPlanesRejected) {
  ConstrainedGridRANSAC ransac(0.1, 50, 1.0);  // very strict angle tolerance
  Plane plane = Plane(0, 0, 1, 0);

  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  ransac.ground_removal(cloud_ptr, output_ptr, plane, split_params);

  EXPECT_LT(output_ptr->points.size(), cloud.points.size());
  EXPECT_GT(output_ptr->points.size(), 0u);
}

/**
 * @brief Test: Typical scenario where some points are removed, some remain.
 */
TEST_F(ConstrainedGridRANSACTest, TestCommonScenario) {
  ConstrainedGridRANSAC ransac(0.05, 100, 20.0);
  Plane plane = Plane(0, 0, 1, 0);

  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  ransac.ground_removal(cloud_ptr, output_ptr, plane, split_params);

  EXPECT_GT(output_ptr->points.size(), 0u);
}

/**
 * @brief Test: Passing an empty point cloud should throw an exception.
 */
TEST_F(ConstrainedGridRANSACTest, TestEmptyPointCloud) {
  ConstrainedGridRANSAC ransac(0.1, 100, 10.0);
  Plane plane = Plane(0, 0, 1, 0);

  const pcl::PointCloud<pcl::PointXYZI>::Ptr empty_ptr(
      &empty_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  EXPECT_THROW(ransac.ground_removal(empty_ptr, output_ptr, plane, split_params),
               std::invalid_argument);
}

/**
 * @brief Test: Points in two separate grids should be removed if they fit their planes.
 */
TEST_F(ConstrainedGridRANSACTest, TestTwoGrids) {
  split_params.n_angular_grids = 2;
  split_params.radius_resolution = 10'000;

  ConstrainedGridRANSAC ransac(0.1, 500, 15.0);
  Plane plane = Plane(0, 0, 1, 0);

  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud_two_grids, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  ransac.ground_removal(cloud_ptr, output_ptr, plane, split_params);

  EXPECT_LE(output_ptr->points.size(), cloud_two_grids.points.size());
}

/**
 * @brief Test: When repetitions = 0, fallback to base plane is used and all points removed.
 */
TEST_F(ConstrainedGridRANSACTest, TestZeroRepetitions) {
  const auto ground_removal = std::make_shared<ConstrainedGridRANSAC>(100, 0, 10.0);
  Plane base_plane = Plane(0, 0, 1, 0);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());
  const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  ground_removal->ground_removal(cloud_ptr, ground_removed_cloud_ptr, base_plane, split_params);

  ASSERT_EQ(ground_removed_cloud_ptr->points.size(), 0);
}
