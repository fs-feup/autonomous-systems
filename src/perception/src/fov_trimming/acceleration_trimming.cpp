#include "fov_trimming/acceleration_trimming.hpp"

AccelerationTrimming::AccelerationTrimming(const TrimmingParameters params) { params_ = params; }

SplitParameters AccelerationTrimming::fov_trimming(
    const pcl::PointCloud<PointXYZIR>::Ptr cloud) const {
  const std::unique_ptr<pcl::PointCloud<PointXYZIR>> trimmed_cloud =
      std::make_unique<pcl::PointCloud<PointXYZIR>>();

  for (auto& point : cloud->points) {
    if (within_limits(point, params_, params_.acc_max_range, params_.acc_fov_trim_angle) &&
        point.y < params_.acc_max_y && point.y > -params_.acc_max_y) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
  return params_.acc_split_params;
}