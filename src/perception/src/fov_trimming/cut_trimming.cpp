#include "fov_trimming/cut_trimming.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

CutTrimming::CutTrimming(const TrimmingParameters params) : FovTrimming(params) {}

SplitParameters CutTrimming::fov_trimming(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  auto& points = cloud->points;
  const size_t n = points.size();

  if (current_mission_type_ != common_lib::competition_logic::Mission::TRACKDRIVE) {
    compute_rotation_constants(params_.acc_max_range, params_.acc_fov_trim_angle);
    current_mission_type_ = common_lib::competition_logic::Mission::TRACKDRIVE;
  }

  size_t out_idx = 0;
  points.reserve(n);  // Reserve space to avoid reallocations

  for (size_t i = 0; i < n; ++i) {
    auto& cur_point = points[i];

    if (within_limits(cur_point)) {
      points[out_idx++] = cur_point;
    }
  }

  points.resize(out_idx);
  cloud->width = static_cast<uint32_t>(out_idx);
  cloud->height = 1;
  cloud->is_dense = true;

  return params_.split_params;
}