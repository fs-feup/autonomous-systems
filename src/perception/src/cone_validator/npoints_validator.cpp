#include <cone_validator/npoints_validator.hpp>

NPointsValidator::NPointsValidator(long unsigned int min_n_points) : _min_n_points_(min_n_points) {}

std::vector<double> NPointsValidator::coneValidator(Cluster* cone_point_cloud,
                                                    [[maybe_unused]] Plane& plane) const {
  // index 0 = if below minimum number of points 0, else 1. No confidence implemented to further
  // punish small clusters.
  if (cone_point_cloud->get_point_cloud()->size() >= (double)_min_n_points_)
    return {1.0};
  else
    return {0.0};
}