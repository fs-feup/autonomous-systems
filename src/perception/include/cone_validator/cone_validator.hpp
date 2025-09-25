#pragma once

#include <utils/cluster.hpp>
#include <utils/pcl_point_types.hpp>
#include <utils/plane.hpp>

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
   * @return vector with all double values for the respective validator
   */
  virtual std::vector<double> coneValidator(Cluster* cone_point_cloud, Plane& plane) const = 0;
};
