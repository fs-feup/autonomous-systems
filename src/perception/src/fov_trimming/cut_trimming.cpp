#include "fov_trimming/cut_trimming.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

CutTrimming::CutTrimming(double pc_max_range, double pc_min_range, double pc_rlidar_max_height,
                         double fov_trim_angle)
    : pc_max_range(pc_max_range),
      pc_min_range(pc_min_range),
      pc_rlidar_max_height(pc_rlidar_max_height),
      fov_trim_angle(fov_trim_angle) {}

void CutTrimming::fov_trimming(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
  pcl::PointCloud<pcl::PointXYZI>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto& point : cloud->points) {
    // Calculate distance LIDAR on the x0y plane
    double distance = std::sqrt((point.x) * (point.x) + point.y * point.y);

    // Calculate the angle of point in the XY plane in deegres
    double angle = std::atan2(point.y, point.x) * 180 / M_PI;

    if (point.z >= pc_rlidar_max_height ||
        distance <= pc_min_range) {  // Ignore points too close or higher than a
                                     // defined height relative
      continue;                      // to the LIDAR's position
    }

    // Check if the point is within the specified distance and angle range
    if (distance <= pc_max_range && angle >= -fov_trim_angle && angle <= fov_trim_angle) {
      trimmed_cloud->points.push_back(point);
    }
  }

  *cloud = *trimmed_cloud;
}