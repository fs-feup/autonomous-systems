#include "fov_trimming/skidpad_trimming.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

SkidpadTrimming::SkidpadTrimming(const TrimmingParameters params) { params_ = params; }

void SkidpadTrimming::fov_trimming(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
  const std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> trimmed_cloud =
      std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();

  // Calculate fov_trim_angle from the given minimum distance to a cone.
  double skid_fov_trim_angle =
      90 - std::acos(1.5 / std::max(params_.skid_min_distance_to_cone, 1.5)) * 180 / M_PI;

  for (auto& point : cloud->points) {
    if (within_limits(point, params_, params_.skid_max_range, skid_fov_trim_angle)) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
}