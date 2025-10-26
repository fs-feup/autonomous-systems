#include "fov_trimming/cut_trimming.hpp"

CutTrimming::CutTrimming(const TrimmingParameters params) { params_ = params; }

SplitParameters CutTrimming::fov_trimming(const pcl::PointCloud<PointXYZIR>::Ptr cloud) const {
  const std::unique_ptr<pcl::PointCloud<PointXYZIR>> trimmed_cloud =
      std::make_unique<pcl::PointCloud<PointXYZIR>>();

  for (auto& point : cloud->points) {
    if (within_limits(point, params_, params_.max_range, params_.fov_trim_angle)) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
  return params_.split_params;
}