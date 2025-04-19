#include <cone_validator/height_validator.hpp>

HeightValidator::HeightValidator(double min_height, double large_max_height,
                                 double small_max_height, double height_cap)
    : _min_height_(min_height),
      _large_max_height_(large_max_height),
      _small_max_height_(small_max_height),
      _height_cap_(height_cap) {}

void HeightValidator::coneValidator(Cluster *cone_point_cloud, EvaluatorResults *results,
                                    Plane &plane) const {
  double maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[0]);
  auto maxPoint = cone_point_cloud->get_point_cloud()->points[0];

  for (long unsigned int i = 0; i < cone_point_cloud->get_point_cloud()->points.size(); i++) {
    if (plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]) > maxZ) {
      maxZ = plane.get_distance_to_point(cone_point_cloud->get_point_cloud()->points[i]);
      maxPoint = cone_point_cloud->get_point_cloud()->points[i];
    }
  }

  double out_ratio_small, out_ratio_large = 1;
  double in_ratio_small, in_ratio_large = 1;
  bool large = false;
  if (maxZ > _small_max_height_) {
    large = true;
    out_ratio_large =
        std::min({_large_max_height_ / maxZ, _min_height_ > 0 ? maxZ / _min_height_ : 1.0, 1.0});
    in_ratio_large = out_ratio_large == 1.0 ? maxZ / _large_max_height_ : 0.0;
  }

  out_ratio_small =
      std::min({_small_max_height_ / maxZ, _min_height_ > 0 ? maxZ / _min_height_ : 1.0, 1.0});
  in_ratio_small = out_ratio_small == 1.0 ? maxZ / _small_max_height_ : 0.0;

  out_ratio_small = out_ratio_small >= _height_cap_ ? out_ratio_small : 0.0;
  out_ratio_large = out_ratio_large >= _height_cap_ ? out_ratio_large : 0.0;

  results->height_out_ratio_large = out_ratio_large;
  results->height_in_ratio_large = in_ratio_large;
  results->height_out_ratio_small = out_ratio_small;
  results->height_in_ratio_small = in_ratio_small;
  results->height_large = large;
}