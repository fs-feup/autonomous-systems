#include <cone_validator/size_validator.hpp>

SizeValidator::SizeValidator(double min_distance_x, double min_distance_y, double min_distance_z)
    : _min_distance_x_(min_distance_x),
      _min_distance_y_(min_distance_y),
      _min_distance_z_(min_distance_z) {}

bool SizeValidator::coneValidator(Cluster* cone_point_cloud, [[maybe_unused]] Plane& plane) const {
  double minX = abs(cone_point_cloud->get_point_cloud()->points[0].x);
  double maxX = abs(cone_point_cloud->get_point_cloud()->points[0].x);

  double minY = abs(cone_point_cloud->get_point_cloud()->points[0].y);
  double maxY = abs(cone_point_cloud->get_point_cloud()->points[0].y);

  double minZ = abs(cone_point_cloud->get_point_cloud()->points[0].z);
  double maxZ = abs(cone_point_cloud->get_point_cloud()->points[0].z);

  for (const auto& point : *cone_point_cloud->get_point_cloud()) {
    if (abs(point.x) > maxX)
      maxX = abs(point.x);
    else if (abs(point.x) < minX)
      minX = abs(point.x);

    if (abs(point.y) > maxY)
      maxY = abs(point.y);
    else if (abs(point.y) < minY)
      minY = abs(point.y);

    if (abs(point.z) > maxZ)
      maxZ = abs(point.z);
    else if (abs(point.z) < minZ)
      minZ = abs(point.z);
  }

  return (maxX - minX >= _min_distance_x_) && (maxY - minY >= _min_distance_y_) &&
         (maxZ - minZ >= _min_distance_z_);
}