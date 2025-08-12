#pragma once

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/split_parameters.hpp>
#include <utils/trimming_parameters.hpp>

#include "common_lib/competition_logic/mission_logic.hpp"

class FovTrimming {
public:
  /**
   * @brief Constructor for FovTrimming class.
   * @param params Trimming parameters containing configuration for the FOV trimming.
   */
  explicit FovTrimming(const std::shared_ptr<TrimmingParameters> params);

  /**
   * @brief Perform fov trimming on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed (trimmed).
   * @param[out] ret The resulting point cloud after trimming and the corresponding split parameters
   * for GridRANSAC.
   */
  virtual SplitParameters fov_trimming(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) = 0;

  void process_point(pcl::PointXYZI& point, double& distance, double& angle) const;

  bool within_limits(pcl::PointXYZI& point) const;

  void set_lidar_rotation(const double rotation);

  void set_lidar_pitch(const double rotation);

  void compute_rotation_constants(double max_range, double fov_trim_angle);

protected:
  const std::shared_ptr<TrimmingParameters> params_;

  double height_limit_;
  double squared_min_range_;
  double squared_max_range_;

  double rot_rad_;
  double cos_rot_;
  double sin_rot_;

  double pitch_rad_;
  double cos_pitch_;
  double sin_pitch_;

  double fov_angle_rad_;
  double min_angle_;
  double max_angle_;
};