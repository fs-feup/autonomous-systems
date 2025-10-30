#include "fov_trimming/acceleration_trimming.hpp"

AccelerationTrimming::AccelerationTrimming(const TrimmingParameters params) { params_ = params; }

void AccelerationTrimming::fov_trimming(
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

  auto& cloud_data = cloud->data;
  size_t num_points = cloud->width * cloud->height;

  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[PointY(i)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[PointZ(i)]);

    if (x == 0.0 && y == 0.0 && z == 0.0) continue;

    if (y < params_.acc_max_y && y > -params_.acc_max_y &&
        within_limits(x, y, z, params_, params_.acc_max_range)) {
      std::memcpy(&trimmed_cloud->data[trimmed_cloud->width * POINT_STEP], &cloud_data[PointX(i)],
                  POINT_STEP);
      trimmed_cloud->width++;
    }
  }

  trimmed_cloud->data.resize(trimmed_cloud->width * POINT_STEP);  // Shrink to actual size
  trimmed_cloud->row_step = trimmed_cloud->width * POINT_STEP;
}
