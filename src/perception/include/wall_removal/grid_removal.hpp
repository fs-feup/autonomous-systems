#pragma once
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/grid_geometry.hpp"
#include "utils/grid_index.hpp"
#include "wall_removal/wall_removal.hpp"

/**
 * @class GridWallRemoval
 *
 * @brief Wall removal implementation using a grid-based approach.
 *
 */
class GridWallRemoval : public WallRemoval {
public:
  /**
   * @brief Constructor for GridWallRemoval
   *
   * @param angle Angular resolution
   * @param radius Base radial bin size
   * @param start_augmentation Distance at which augmentation starts
   * @param radius_augmentation Bin size increase per bin
   * @param fov Field of view
   * @param max_points_per_cluster Maximum number of points allowed per each grid to be considered
   */
  GridWallRemoval(double angle, double radius, double start_augmentation,
                  double radius_augmentation, double fov, int max_points_per_cluster);

  /**
   * @brief Removes walls from the input point cloud using a grid-based approach.
   *
   * @param point_cloud The input point cloud
   * @param output_cloud The output point cloud with walls and big objects removed
   */
  void remove_walls(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                    sensor_msgs::msg::PointCloud2::SharedPtr& output_cloud) const override;

private:
  GridGeometry grid_geometry_;  // Grid geometry for grid calculations
  int max_points_per_cluster_;  // Maximum number of points allowed per each grid to be considered
                                // non-wall
};