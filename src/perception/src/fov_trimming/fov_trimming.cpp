#include "fov_trimming/fov_trimming.hpp"

void FovTrimming::process_point(pcl::PointXYZI& point, double& distance, double& angle) const {
  const double x = point.x;
  const double y = point.y;

  // This rotates 90º:
  point.x = -y;
  point.y = x;

  // Calculate distance LIDAR on the x0y plane
  distance = std::sqrt((point.x) * (point.x) + point.y * point.y);

  // Calculate the angle of point in the XY plane in deegres
  angle = std::atan2(point.y, point.x) * 180 / M_PI;
}