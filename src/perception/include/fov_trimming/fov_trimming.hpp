#pragma once

#include <cstring>
#include <utils/lidar_point.hpp>
#include <utils/trimming_parameters.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

class FovTrimming {
public:
  /**
   * @brief Perform fov trimming on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed (trimmed).
   * @param[out] ret The resulting point cloud after trimming and the corresponding split parameters
   * for GridRANSAC.
   */
  virtual void fov_trimming(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                            sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_cloud) const = 0;

  bool within_limits(float x, float y, float z, const TrimmingParameters& params,
                     const double max_range) const;

protected:
  TrimmingParameters params_;
};