#include "fov_trimming/acceleration_trimming.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

AccelerationTrimming::AccelerationTrimming(const TrimmingParameters params) { params_ = params; }

PclSplitParameters AccelerationTrimming::fov_trimming(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
  const std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> trimmed_cloud =
      std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();

  for (auto& point : cloud->points) {
    if (within_limits(point, params_, params_.acc_max_range, params_.acc_fov_trim_angle) &&
        point.y < params_.acc_max_y && point.y > -params_.acc_max_y) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
  return params_.acc_split_params;
}