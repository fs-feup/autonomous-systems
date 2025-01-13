#include <cone_validator/npoints_validator.hpp>

NPointsValidator::NPointsValidator(long unsigned int min_n_points) : _min_n_points_(min_n_points) {}

bool NPointsValidator::coneValidator(Cluster* cone_point_cloud,
                                     [[maybe_unused]] Plane& plane) const {
  return cone_point_cloud->get_point_cloud()->size() >= _min_n_points_;
}