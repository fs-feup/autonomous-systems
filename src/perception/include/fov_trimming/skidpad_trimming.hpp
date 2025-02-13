#pragma once

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fov_trimming/fov_trimming.hpp"

class SkidpadTrimming : public FovTrimming {
private:
  double pc_max_range = 30.2853;
  double pc_min_range, pc_rlidar_max_height, min_distance_to_cone, fov_trim_angle;

public:
  /**
   * @brief Constructor for the Skidpad Point Cloud Trimming algorithm.
   * @param pc_min_range Minimum point cloud distance after trimming.
   * @param pc_rlidar_max_height Maximum point cloud height after trimming.
   * @param  min_distance_to_cone Minimum distance to a cone for it to be seen.
   */
  SkidpadTrimming(double pc_min_range, double pc_rlidar_max_height, double min_distance_to_cone);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  SkidpadTrimming() = default;

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