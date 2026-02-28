#include "ground_removal/himmelsbach.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <test_utils/pointcloud2_helper.hpp>
#include <utils/ground_grid.hpp>

class HimmelsbachTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Points: x, y, z, intensity, ring
    ground_pts = {{1.0, 0.0, -1.0, 0.5, 39}, {2.0, 0.0, -1.2, 0.5, 38}, {3.0, 0.0, -1.1, 0.5, 37}};
    non_ground_pts = {{1.0, 0.0, 1.0, 0.5, 39}, {2.0, 0.0, 1.2, 0.5, 38}, {3.0, 0.0, 1.1, 0.5, 37}};
    mixed_pts = {{1.0, 0.0, -1.0, 0.5, 39},
                 {2.0, 0.0, 1.2, 0.5, 38},
                 {3.0, 0.0, -1.1, 0.5, 37},
                 {4.0, 0.0, 1.3, 0.5, 37}};
    empty_pts = {};

    grid_angle = 0.1;
    max_slope = 0.3;
    initial_alpha = 0.2;
    alpha_augmentation_m = 0.05;
    start_augmentation = 10.0;

    trim_params = TrimmingParameters();
    trim_params.lidar_height = 1.0;
    trim_params.fov = 180.0;
    trim_params.lidar_horizontal_resolution = 0.1;
    trim_params.lidar_vertical_resolution = 0.1;
    trim_params.apply_rotation = false;
    trim_params.rotation = 0.0;
    trim_params.apply_fov_trimming = false;
    trim_params.max_height = 2.0;
    trim_params.min_range = 0.0;
    trim_params.max_range = 100.0;
    ground_grid = std::make_unique<GroundGrid>(30.0, 0.1, 0.5, 10.0, 0.1, 3.14);
    algo = std::make_unique<Himmelsbach>(grid_angle, max_slope, initial_alpha, alpha_augmentation_m,
                                         start_augmentation, trim_params);
  }

  std::vector<std::array<float, 5>> ground_pts;
  std::vector<std::array<float, 5>> non_ground_pts;
  std::vector<std::array<float, 5>> mixed_pts;
  std::vector<std::array<float, 5>> empty_pts;
  double grid_angle, max_slope, initial_alpha, alpha_augmentation_m, start_augmentation;
  TrimmingParameters trim_params;
  std::unique_ptr<GroundGrid> ground_grid;
  std::unique_ptr<Himmelsbach> algo;
};

/**
 * @brief An empty input cloud should result in an empty output cloud.
 */
TEST_F(HimmelsbachTest, EmptyCloudReturnsEmpty) {
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(empty_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  algo->ground_removal(cloud_ptr, ground_removed_cloud_ptr, *ground_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}

/**
 * @brief All points are below ground threshold and should be removed.
 */
TEST_F(HimmelsbachTest, AllGroundPointsRemoved) {
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(ground_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  algo->ground_removal(cloud_ptr, ground_removed_cloud_ptr, *ground_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 0);
}

/**
 * @brief All points are above ground threshold and should be retained.
 */
TEST_F(HimmelsbachTest, AllNonGroundPointsRetained) {
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(non_ground_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  algo->ground_removal(cloud_ptr, ground_removed_cloud_ptr, *ground_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, non_ground_pts.size());
}

/**
 * @brief Mixed input; only non-ground points should be retained in the output.
 */
TEST_F(HimmelsbachTest, MixedPointsOnlyNonGroundRetained) {
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(mixed_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  algo->ground_removal(cloud_ptr, ground_removed_cloud_ptr, *ground_grid);
  ASSERT_EQ(ground_removed_cloud_ptr->width, 2);
}

/**
 * @brief GroundGrid is reset and updated after ground removal.
 */
TEST_F(HimmelsbachTest, GroundGridIsResetAndUpdated) {
  auto cloud_ptr = test_utils::make_lidar_pointcloud2(ground_pts);
  auto ground_removed_cloud_ptr = test_utils::make_lidar_pointcloud2({});
  ground_grid->set_ground_height(1.0, 0.0, 99.0f);
  algo->ground_removal(cloud_ptr, ground_removed_cloud_ptr, *ground_grid);

  float h = ground_grid->get_ground_height(1.0f, 0.0f);
  ASSERT_NE(h, 99.0f);
}
