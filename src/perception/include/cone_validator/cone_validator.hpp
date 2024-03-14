#ifndef CONE_VALIDATOR_HPP
#define CONE_VALIDATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/cluster.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

class ConeValidator {
 public:
  virtual bool coneValidator(Cluster* cone_point_cloud) const = 0;
};

#endif  // CONE_VALIDATOR_HPP