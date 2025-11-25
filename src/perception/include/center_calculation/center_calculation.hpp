#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <utils/lidar_point.hpp>
#include <utils/plane.hpp>

/**
 * @brief Abstract Class representing the logic for estimating the center position of a cone.
 *
 * This class provides an interface for various methods to estimate the center position of a cone
 * based on its point cloud data, optionally considering a ground plane.
 */
class CenterCalculator {
public:
  /**
   * @brief Estimates the center position of a cone.
   *
   * This method takes a point cloud representing a cone's cluster and optionally a ground plane
   * to refine the center estimation. The center position is returned as a 4-dimensional vector
   * (Eigen::Vector4f).
   *
   * @param point_cloud Pointer to the point cloud representing the cone's cluster.
   * @param plane Plane representing the ground (optional). Default is an empty Plane object.
   * @return Eigen::Vector4f representing the estimated center of the cone.
   */
  virtual Eigen::Vector4f calculate_center(
      const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
      const std::vector<int>& point_indices, const Plane& plane = Plane()) const = 0;
};
