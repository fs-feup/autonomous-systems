#include "fov_trimming/cut_trimming.hpp"

#include <cstring>

CutTrimming::CutTrimming(const TrimmingParameters params) { params_ = params; }

void CutTrimming::fov_trimming(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
                               sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_cloud) const {
  // Copy metadata
  trimmed_cloud->header = cloud->header;
  trimmed_cloud->height = 1;
  trimmed_cloud->is_dense = false;
  trimmed_cloud->fields = cloud->fields;
  trimmed_cloud->point_step = cloud->point_step;
  trimmed_cloud->width = 0;
  trimmed_cloud->row_step = 0;
  trimmed_cloud->data.resize(cloud->data.size());

  const auto& data = cloud->data;
  const size_t n = cloud->width * cloud->height;

  const bool rotate = params_.apply_rotation;

  double cos_t = 1.0;
  double sin_t = 0.0;
  if (rotate) {
    const double theta = params_.rotation * M_PI / 180.0;
    cos_t = std::cos(theta);
    sin_t = std::sin(theta);
  }

  for (size_t i = 0; i < n; ++i) {
    const float x = *reinterpret_cast<const float*>(&data[LidarPoint::PointX(i)]);
    const float y = *reinterpret_cast<const float*>(&data[LidarPoint::PointY(i)]);
    const float z = *reinterpret_cast<const float*>(&data[LidarPoint::PointZ(i)]);

    // Skip invalid points
    if (x == 0.0f && y == 0.0f && z == 0.0f) {
      continue;
    }

    float rx = x;
    float ry = y;

    if (rotate) {
      rx = static_cast<float>(x * cos_t - y * sin_t);
      ry = static_cast<float>(x * sin_t + y * cos_t);
    }

    if (within_limits(rx, ry, z, params_, params_.max_range)) {
      uint8_t* out = &trimmed_cloud->data[trimmed_cloud->width * LidarPoint::POINT_STEP];

      std::memcpy(out, &data[LidarPoint::PointX(i)], LidarPoint::POINT_STEP);

      // Overwrite X/Y only if rotation is active
      if (rotate) {
        *reinterpret_cast<float*>(out + LidarPoint::PointX(0)) = rx;
        *reinterpret_cast<float*>(out + LidarPoint::PointY(0)) = ry;
      }

      trimmed_cloud->width++;
    }
  }

  trimmed_cloud->data.resize(trimmed_cloud->width * LidarPoint::POINT_STEP);
  trimmed_cloud->row_step = trimmed_cloud->width * LidarPoint::POINT_STEP;
}
