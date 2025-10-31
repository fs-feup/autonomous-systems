#pragma once

#include <center_calculation/center_calculation.hpp>
#include <cmath>
#include <limits>
#include <string>

/**
 * @brief Concrete class representing a circumference-based method for estimating the center
 * position of a cone.
 *
 * This class implements the CenterCalculator interface to provide a method for estimating the
 * center position of a cone based on its point cloud data, using a circumference-based approach.
 */
class CircunferenceCenterCalculator : public CenterCalculator {
public:
  /**
   * @brief Estimates the center position of a cone using a circumference-based approach.
   *
   * This method estimates the center position of a cone by approximating its center as the center
   * of a circumference that cuts the cone.
   *
   * @param point_cloud Pointer to the point cloud representing the cone's cluster.
   * @param plane Plane representing the ground. (Mandatory)
   * @return Eigen::Vector4f representing the estimated center of the cone.
   *
   * @note The plane parameter is mandatory for this method.
   */
  Eigen::Vector4f calculate_center(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                                   const std::vector<int>& point_indices,
                                   const Plane& plane = Plane()) const override;
};
