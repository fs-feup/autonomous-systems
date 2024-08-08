#include <cone_validator/height_validator.hpp>

HeightValidator::HeightValidator(double height) : height(height) {}

bool HeightValidator::cone_validator(Cluster* cone_point_cloud, Plane& plane) const {
  double maxZ = cone_point_cloud->get_point_cloud()->points[0].z;
  auto maxPoint = cone_point_cloud->get_point_cloud()->points[0];

  for (long unsigned int i = 0; i < cone_point_cloud->get_point_cloud()->points.size(); i++) {
    if (cone_point_cloud->get_point_cloud()->points[i].z > maxZ) {
      maxZ = cone_point_cloud->get_point_cloud()->points[i].z;
      maxPoint = cone_point_cloud->get_point_cloud()->points[i];
    }
  }

  return plane.get_distance_to_point(maxPoint) < height;
}