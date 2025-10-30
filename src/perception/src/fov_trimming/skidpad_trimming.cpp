#include "fov_trimming/skidpad_trimming.hpp"

SkidpadTrimming::SkidpadTrimming(const TrimmingParameters params) { params_ = params; }

SplitParameters SkidpadTrimming::fov_trimming(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_cloud) const {
  // Copy header and field layout
  trimmed_cloud->header = cloud->header;
  trimmed_cloud->height = 1;
  trimmed_cloud->is_dense = false;
  trimmed_cloud->fields = cloud->fields;
  trimmed_cloud->point_step = cloud->point_step;
  trimmed_cloud->row_step = 0;
  trimmed_cloud->width = 0;
  trimmed_cloud->data.resize(cloud->data.size());

  const uint8_t* cloud_data = cloud->data.data();
  size_t num_points = cloud->width * cloud->height;

  size_t offset = 0;

  for (size_t i = 0; i < num_points; ++i) {
    LidarPoint p(i);

    float x = *reinterpret_cast<const float*>(&cloud_data[p.x()]);
    float y = *reinterpret_cast<const float*>(&cloud_data[p.y()]);
    float z = *reinterpret_cast<const float*>(&cloud_data[p.z()]);

    if (x == 0.0 && y == 0.0 && z == 0.0) continue;

    if (within_limits(x, y, z, params_, params_.skid_max_range)) {
      std::memcpy(&trimmed_cloud->data[offset], cloud_data + i * LidarPoint::POINT_STEP,
                  LidarPoint::POINT_STEP);
      offset += LidarPoint::POINT_STEP;
      trimmed_cloud->width++;
    }
  }

  trimmed_cloud->data.resize(offset);  // Shrink to actual size
  trimmed_cloud->row_step = trimmed_cloud->width * trimmed_cloud->point_step;

  return params_.split_params;
}
