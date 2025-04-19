#include <cone_validator/npoints_validator.hpp>

NPointsValidator::NPointsValidator(long unsigned int min_n_points) : _min_n_points_(min_n_points) {}

void NPointsValidator::coneValidator(Cluster* cone_point_cloud, EvaluatorResults* results,
                                     [[maybe_unused]] Plane& plane) const {
  // No confidence implemented to further punish small clusters.
  results->n_points = cone_point_cloud->get_point_cloud()->size() >= (double)_min_n_points_;
}