#pragma once
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "utils/lidar_point.hpp"

/**
 * @brief Abstract base class for wall removal of point clouds.
 *
 * The WallRemoval class defines an interface for removing walls from point clouds.
 * Subclasses must implement the pure virtual function remove_walls().
 */
class WallRemoval {
public:
  /**
   * @brief Removes walls from the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud
   * @param output_cloud The output point cloud with walls and big objects removed
   */
  virtual void remove_walls(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                            sensor_msgs::msg::PointCloud2::SharedPtr& output_cloud) = 0;
};