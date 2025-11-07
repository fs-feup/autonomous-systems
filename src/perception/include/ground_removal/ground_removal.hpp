#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utils/ground_grid.hpp>
#include <utils/lidar_point.hpp>
#include <utils/trimming_parameters.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * @class GroundRemoval
 * @brief Abstract class for ground removal from a point cloud.
 *
 * This class defines an interface for ground removal algorithms from a point cloud.
 * Inherit from this class to implement specific ground removal algorithms.
 */
class GroundRemoval {
public:
  /**
   * @brief Perform ground removal on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed.
   * @param[out] ret The resulting point cloud after ground removal.
   */
  virtual void ground_removal(const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
                              sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud,
                              GroundGrid& ground_grid) const = 0;
};
