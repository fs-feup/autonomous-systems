#pragma once

#include <Eigen/Dense>
#include <utils/cluster.hpp>
#include <utils/pcl_point_types.hpp>

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
