#include "fov_trimming/skidpad_trimming.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

SkidpadTrimming::SkidpadTrimming(const TrimmingParameters params) { params_ = params; }

SplitParameters SkidpadTrimming::fov_trimming(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
  const std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> trimmed_cloud =
      std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();

  for (auto& point : cloud->points) {
    if (within_limits(point, params_, params_.skid_max_range, params_.skid_fov_trim_angle)) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
  return params_.skid_split_params;
}