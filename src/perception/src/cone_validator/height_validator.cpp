#include <cone_validator/height_validator.hpp>

HeightValidator::HeightValidator(double min_height, double large_max_height,
                                 double small_max_height)
    : _min_height_(min_height),
      _large_max_height_(large_max_height),
      _small_max_height_(small_max_height) {}

bool HeightValidator::coneValidator(Cluster* cone_point_cloud, Plane& plane) const {
  double maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[0]);
  auto maxPoint = cone_point_cloud->get_point_cloud()->points[0];

  for (long unsigned int i = 0; i < cone_point_cloud->get_point_cloud()->points.size(); i++) {
    if (plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]) > maxZ) {
      maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]);
      maxPoint = cone_point_cloud->get_point_cloud()->points[i];
    }
  }

  if (plane.get_distance_to_point(maxPoint) > _small_max_height_) cone_point_cloud->set_is_large();

  return plane.get_distance_to_point(maxPoint) < _large_max_height_ &&
         plane.get_distance_to_point(maxPoint) > _min_height_;
}