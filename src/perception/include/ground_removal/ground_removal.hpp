#pragma once

#include <rclcpp/rclcpp.hpp>
#include <utils/pcl_point_types.hpp>
#include <utils/plane.hpp>
#include <utils/split_parameters.hpp>

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
  virtual void ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                              const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                              const SplitParameters split_params) const = 0;
};
