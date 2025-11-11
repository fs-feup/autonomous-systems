#pragma once
#include <center_calculation/center_calculation.hpp>

/**
 * @brief Concrete Class representing a centroid-based method for estimating the center position of
 * a cone.
 *
 * This class implements the CenterCalculator interface to provide a method for estimating the
 * center position of a cone based on its point cloud data, using centroid calculation.
 */
class CentroidCalculator : public CenterCalculator {
public:
  /**
   * @brief Estimates the center position of a cone using centroid calculation.
   *
   * This method estimates the center position of a cone by calculating the centroid of its point
   * cloud data.
   *
   * @param point_cloud Pointer to the point cloud representing the cone's cluster.
   * @param plane Plane representing the ground [Not used in this concrete implementation]
   * @return Eigen::Vector4f representing the estimated center of the cone.
   */
  Eigen::Vector4f calculate_center(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                                   const std::vector<int>& point_indices,
                                   [[maybe_unused]] const Plane& plane = Plane()) const override;
};
