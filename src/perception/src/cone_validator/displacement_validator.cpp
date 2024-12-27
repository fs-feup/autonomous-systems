#include <cone_validator/displacement_validator.hpp>

DisplacementValidator::DisplacementValidator(double min_distance_x, double min_distance_y,
                                             double min_distance_z)
    : _min_distance_x_(min_distance_x),
      _min_distance_y_(min_distance_y),
      _min_distance_z_(min_distance_z) {}

std::vector<double> DisplacementValidator::coneValidator(Cluster* cone_point_cloud,
                                                         [[maybe_unused]] Plane& plane) const {
  float minX = abs(cone_point_cloud->get_point_cloud()->points[0].x);
  float maxX = abs(cone_point_cloud->get_point_cloud()->points[0].x);

  float minY = abs(cone_point_cloud->get_point_cloud()->points[0].y);
  float maxY = abs(cone_point_cloud->get_point_cloud()->points[0].y);

  float minZ = abs(cone_point_cloud->get_point_cloud()->points[0].z);
  float maxZ = abs(cone_point_cloud->get_point_cloud()->points[0].z);

  for (long unsigned int i = 1; i < cone_point_cloud->get_point_cloud()->points.size(); i++) {
    pcl::PointXYZI point = cone_point_cloud->get_point_cloud()->points[i];

    minX = std::min(minX, abs(point.x));
    maxX = std::max(maxX, abs(point.x));

    minY = std::min(minY, abs(point.y));
    maxY = std::max(maxY, abs(point.y));

    minZ = std::min(minZ, abs(point.z));
    maxZ = std::max(maxZ, abs(point.z));
  }

  // index 0 = ratio between the x axis displacement and the minimum distance for that axis.
  // index 1 = ratio between the y axis displacement and the minimum distance for that axis.
  // index 2 = ratio between the z axis displacement and the minimum distance for that axis.
  return {std::min(maxX - minX / _min_distance_x_, 1.0),
          std::min(maxY - minY / _min_distance_y_, 1.0),
          std::min(maxZ - minZ / _min_distance_z_, 1.0)};
}