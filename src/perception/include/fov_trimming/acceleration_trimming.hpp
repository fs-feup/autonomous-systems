#pragma once

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fov_trimming/fov_trimming.hpp"

class AccelerationTrimming : public FovTrimming {
private:
  double pc_max_range = 20.250, fov_trim_angle = 90;
  double pc_max_y, pc_min_range, pc_rlidar_max_height;

public:
  /**
   * @brief Constructor for the Acceleration Point Cloud Trimming algorithm.
   * @param pc_max_y Maximum lateral distance after trimming.
   * @param pc_min_range Minimum point cloud distance after trimming.
   * @param pc_rlidar_max_height Maximum point cloud height after trimming.
   */
  AccelerationTrimming(double pc_min_range, double pc_rlidar_max_height, double pc_max_y);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  AccelerationTrimming() = default;

  /**
   * @brief Perform ground removal on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed (trimmed).
   * @param[out] ret The resulting point cloud after trimming.
   */
  void fov_trimming(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) const override;
};