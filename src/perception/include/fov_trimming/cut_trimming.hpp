#pragma once

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fov_trimming/fov_trimming.hpp"

class CutTrimming : public FovTrimming {
private:
  double pc_max_range, pc_min_range, pc_rlidar_max_height;
  double fov_trim_angle;

public:
  /**
   * @brief Constructor for the Cut Point Cloud Trimming algorithm.
   * @param pc_max_range Maximum point cloud distance after trimming.
   * @param pc_min_range Minimum point cloud distance after trimming.
   * @param pc_rlidar_max_height Maximum point cloud height after trimming.
   * @param fov_trim_angle Maximum point cloud angle on both sides after trimming.
   */
  CutTrimming(double pc_max_range, double pc_min_range, double pc_rlidar_max_height,
              double fov_trim_angle);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  CutTrimming() = default;

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