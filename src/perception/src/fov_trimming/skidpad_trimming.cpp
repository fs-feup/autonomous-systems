#include "fov_trimming/skidpad_trimming.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

SkidpadTrimming::SkidpadTrimming(double pc_min_range, double pc_rlidar_max_height,
                                 double min_distance_to_cone)
    : pc_min_range(pc_min_range),
      pc_rlidar_max_height(pc_rlidar_max_height),
      min_distance_to_cone(min_distance_to_cone) {
  // Calculate fov_trim_angle from the given minimum distance to a cone.
  fov_trim_angle = 90 - std::acos(1.5 / std::max(min_distance_to_cone, 1.5)) * 180 / M_PI;
}

void SkidpadTrimming::fov_trimming(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
  pcl::PointCloud<pcl::PointXYZI>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto& point : cloud->points) {
    double x = point.x;
    double y = point.y;

    // This rotates 90º:
    point.x = -y;
    point.y = x;

    // Calculate distance LIDAR on the x0y plane
    double distance = std::sqrt((point.x) * (point.x) + point.y * point.y);

    // Calculate the angle of point in the XY plane in deegres
    double angle = std::atan2(point.y, point.x) * 180 / M_PI;

    if (point.z >= pc_rlidar_max_height || distance > pc_max_range ||
        distance <= pc_min_range) {  // Ignore points too close, too far, or higher than a
                                     // defined height relative
      continue;                      // to the LIDAR's position
    }

    // Check if the point is within the specified distance, angle range and lateral distance
    if (angle >= -fov_trim_angle && angle <= fov_trim_angle) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
}