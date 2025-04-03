#pragma once

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/split_parameters.hpp>
#include <utils/trimming_parameters.hpp>

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
  virtual SplitParameters fov_trimming(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) const = 0;

  void process_point(pcl::PointXYZI& point, const double rotation, const double pitch,
                     double& distance, double& angle) const;

  bool within_limits(pcl::PointXYZI& point, const TrimmingParameters& params,
                     const double max_range, const double fov_trim_angle) const;

  void set_lidar_rotation(const double rotation);

  void set_lidar_pitch(const double rotation);

protected:
  TrimmingParameters params_;
};