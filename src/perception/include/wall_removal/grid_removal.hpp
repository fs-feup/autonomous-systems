#pragma once
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/grid_index.hpp"
#include "wall_removal/wall_removal.hpp"

/**
 * @brief Abstract base class for wall removal of point clouds.
 *
 * The WallRemoval class defines an interface for removing walls from point clouds.
 * Subclasses must implement the pure virtual function remove_walls().
 */
class GridWallRemoval : public WallRemoval {
private:
  double grid_width_;
  int max_points_per_cluster_;

public:
  GridWallRemoval(double grid_width, int max_points_per_cluster);

  /**
   * @brief Removes walls from the input point cloud using a grid-based approach.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud A shared pointer to a point cloud of type pcl::PointCloud<pcl::PointXYZI>.
   * @param[out] output_cloud A shared pointer to a point cloud to store the resulting point cloud
   * after wall removal.
   */
  void remove_walls(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                    sensor_msgs::msg::PointCloud2::SharedPtr& output_cloud) const override;
};