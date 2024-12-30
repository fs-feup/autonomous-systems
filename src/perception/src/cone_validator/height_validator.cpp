#include <cone_validator/height_validator.hpp>

HeightValidator::HeightValidator(double min_height, double large_max_height,
                                 double small_max_height, double height_cap)
    : _min_height_(min_height),
      _large_max_height_(large_max_height),
      _small_max_height_(small_max_height),
      _height_cap_(height_cap) {}

std::vector<double> HeightValidator::coneValidator(Cluster* cone_point_cloud, Plane& plane) const {
  double maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[0]);
  auto maxPoint = cone_point_cloud->get_point_cloud()->points[0];

  for (long unsigned int i = 0; i < cone_point_cloud->get_point_cloud()->points.size(); i++) {
    if (plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]) > maxZ) {
      maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]);
      maxPoint = cone_point_cloud->get_point_cloud()->points[i];
    }
  }

  std::vector<double> res;
  if (maxZ > _small_max_height_) {
    cone_point_cloud->set_is_large();
    res.push_back(
        std::min({_large_max_height_ / maxZ, _min_height_ > 0 ? maxZ / _min_height_ : 1.0, 1.0}));
    res.push_back(res[0] == 1.0 ? maxZ / _large_max_height_ : 0.0);
  } else {
    res.push_back(
        std::min({_small_max_height_ / maxZ, _min_height_ > 0 ? maxZ / _min_height_ : 1.0, 1.0}));
    res.push_back(res[0] == 1.0 ? maxZ / _small_max_height_ : 0.0);
  }
  res[0] = res[0] >= _height_cap_ ? res[0] : 0.0;
  // index 0 = if not in height interval, ratio between the height of the cluster and the maximum or
  // minimum height, whichever is closest to.
  // index 1 = if in height interval, how close is it to the maximum height, else 0.
  return res;
}