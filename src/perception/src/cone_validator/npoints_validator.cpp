#include <cone_validator/npoints_validator.hpp>

NPointsValidator::NPointsValidator(long unsigned int min_n_points) : _min_n_points_(min_n_points) {}

std::vector<double> NPointsValidator::coneValidator(Cluster* cone_point_cloud,
                                                    [[maybe_unused]] Plane& plane) const {
  // index 0 = if below minimum number of points ratio between cluster point number and minimum
  // points, else 1.
  return {std::min({cone_point_cloud->get_point_cloud()->size() / (double)_min_n_points_, 1.0})};
}