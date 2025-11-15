#pragma once

#include <utils/cluster.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "utils/lidar_point.hpp"

/**
 * @class ConeDifferentiation
 *
 * @brief Abstract class for cone differentiation based on cone reflectivity patterns.
 */
class ConeDifferentiation {
public:
  /**
   * @brief Perform cone differentiation on a cone's point cloud.
   *
   * @param cone_point_cloud Cluster's point cloud.
   * @return Color The cone's color.
   */
  virtual void coneDifferentiation(Cluster* cone_point_cloud) const = 0;
};
