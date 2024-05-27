#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/cluster.hpp>
#include <utils/plane.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * @class ConeValidator
 *
 * @brief Abstract class responsible for validating clusters as cones
 *
 */
class ConeValidator {
 public:
  /**
   * @brief Validate a Cluster as a cone
   *
   * @param cone_point_cloud Point Cloud to be Validated
   * @return true if the cluster is considered a cone
   * @return false if the cluster is not considered a cone
   */
  virtual bool coneValidator(Cluster* cone_point_cloud, Plane& plane) const = 0;
};
