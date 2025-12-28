#include "fov_trimming/cut_trimming.hpp"

#include <cstring>

CutTrimming::CutTrimming(const TrimmingParameters params) { params_ = params; }

void CutTrimming::fov_trimming(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
                               sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_cloud) const {
  // Copy header
  trimmed_cloud->header = cloud->header;
  trimmed_cloud->height = 1;
  trimmed_cloud->is_dense = false;
  trimmed_cloud->fields = cloud->fields;
  trimmed_cloud->point_step = cloud->point_step;
  trimmed_cloud->width = 0;
  trimmed_cloud->row_step = 0;
  trimmed_cloud->data.resize(cloud->data.size());

  const auto& cloud_data = cloud->data;
  const size_t num_points = cloud->width * cloud->height;

  const bool do_rotation = params_.apply_rotation;
  const double theta = params_.rotation * M_PI / 180.0;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  for (size_t i = 0; i < num_points; ++i) {
    const float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(i)]);
    const float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(i)]);
    const float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(i)]);

    // Skip invalid points
    if (x == 0.0f && y == 0.0f && z == 0.0f) {
      continue;
    }

    float rx = x;
    float ry = y;

    if (do_rotation) {
      rx = static_cast<float>(x * cos_theta - y * sin_theta);
      ry = static_cast<float>(x * sin_theta + y * cos_theta);
    }

    if (within_limits(rx, ry, z, params_, params_.max_range)) {
      uint8_t* out = &trimmed_cloud->data[trimmed_cloud->width * LidarPoint::POINT_STEP];
      std::memcpy(out, &cloud_data[LidarPoint::PointX(i)], LidarPoint::POINT_STEP);
      trimmed_cloud->width++;
    }
  }

  // Shrink buffer to actual size
  trimmed_cloud->data.resize(trimmed_cloud->width * LidarPoint::POINT_STEP);
  trimmed_cloud->row_step = trimmed_cloud->width * LidarPoint::POINT_STEP;
}