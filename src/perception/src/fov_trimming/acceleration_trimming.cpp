#include "fov_trimming/acceleration_trimming.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

AccelerationTrimming::AccelerationTrimming(TrimmingParameters params) : params_(params) {
  params_.set_acceleration();
}

void AccelerationTrimming::fov_trimming(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> trimmed_cloud =
      std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();

  double distance, angle;
  for (auto& point : cloud->points) {
    process_point(point, distance, angle);

    if (point.z >= params_.max_height - params_.lidar_height || distance > params_.max_range ||
        distance <= params_.min_range) {  // Ignore points too close, too far, or higher than a
                                          // defined height relative
      continue;                           // to the LIDAR's position
    }

    // Check if the point is within the specified distance, angle range and lateral distance
    if (angle >= -params_.fov_trim_angle && angle <= params_.fov_trim_angle &&
        params_.acc_max_y >= point.y && -params_.acc_max_y <= point.y) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
}