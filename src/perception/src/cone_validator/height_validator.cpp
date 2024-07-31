#include <cone_validator/height_validator.hpp>

HeightValidator::HeightValidator(double min_height, double max_height) : min_height(min_height), max_height(max_height) {}

bool HeightValidator::coneValidator(Cluster* cone_point_cloud, Plane& plane) const {
  double maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[0]);
  auto maxPoint = cone_point_cloud->get_point_cloud()->points[0];

  for (long unsigned int i = 0; i < cone_point_cloud->get_point_cloud()->points.size(); i++) {
    if (plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]) > maxZ) {
      maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]);
      maxPoint = cone_point_cloud->get_point_cloud()->points[i];
    }
  }

  return plane.get_distance_to_point(maxPoint) < max_height && plane.get_distance_to_point(maxPoint) > min_height;
}