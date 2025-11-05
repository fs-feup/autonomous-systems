#include <cone_evaluator/cone_evaluator.hpp>

ConeEvaluator::ConeEvaluator(std::shared_ptr<EvaluatorParameters> params) : params_(params) {}

bool ConeEvaluator::evaluateCluster(Cluster &cluster, Plane &ground_plane) {
  double confidence = 0;
  std::vector<double> cur_validator_results;

  // Cylinder

  cur_validator_results = params_->cylinder_validator->coneValidator(&cluster, ground_plane);
  // index 0 = ratio of between distance to the farthest point and the cylinder radius.
  // index 1 = ratio of between distance to the farthest point and the cylinder heigth.
  // index 2 = ratio between the number of points outside the cylinder and the number of total
  // points.

  confidence += params_->cylinder_radius_weight * cur_validator_results.at(0) +
                params_->cylinder_height_weight * cur_validator_results.at(1) +
                params_->cylinder_npoints_weight * cur_validator_results.at(2);

  // Adjust for precision issues [0,1]
  confidence = std::max(0.0, std::min(confidence, 1.0));

  cluster.set_confidence(confidence);
  return confidence >= params_->min_confidence;
}