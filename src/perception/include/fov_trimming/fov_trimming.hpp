#pragma once

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class FovTrimming {
public:
  /**
   * @brief Perform fov trimming on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed (trimmed).
   * @param[out] ret The resulting point cloud after trimming.
   */
  virtual void fov_trimming(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) const = 0;
};