#include <cone_evaluator/cone_evaluator.hpp>

ConeEvaluator::ConeEvaluator(
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<ConeValidator>>>
        cone_validators,
    std::shared_ptr<std::unordered_map<std::string, double>> evaluator_weights,
    double min_confidence)
    : cone_validators_(cone_validators),
      evaluator_weights_(evaluator_weights),
      min_confidence_(min_confidence) {}

bool ConeEvaluator::evaluateCluster(Cluster &cluster, Plane &ground_plane) {
  double confidence = 0;
  std::vector<double> cur_validator_results;

  // Height

  cur_validator_results = cone_validators_->at("height")->coneValidator(&cluster, ground_plane);
  // index 0 = if not in height interval, ratio between the height of the cluster and the maximum or
  // minimum height, whichever is closest to.
  // index 1 = if in height interval, how close is it to the maximum height, else 0.

  confidence += evaluator_weights_->at("height_out_weight") * cur_validator_results.at(0) +
                evaluator_weights_->at("height_in_weight") * cur_validator_results.at(1);

  // Cylinder

  cur_validator_results = cone_validators_->at("cylinder")->coneValidator(&cluster, ground_plane);
  // index 0 = ratio of between distance to the farthest point and the cylinder radius.
  // index 1 = ratio of between distance to the farthest point and the cylinder heigth.
  // index 2 = ratio between the number of points outside the cylinder and the number of total
  // points.

  confidence += evaluator_weights_->at("cylinder_radius_weight") * cur_validator_results.at(0) +
                evaluator_weights_->at("cylinder_height_weight") * cur_validator_results.at(1) +
                evaluator_weights_->at("cylinder_npoints_weight") * cur_validator_results.at(2);

  // NPoints

  cur_validator_results = cone_validators_->at("npoints")->coneValidator(&cluster, ground_plane);
  // index 0 = if below minimum number of points ratio between cluster point number and minimum
  // points, else 1.

  confidence += evaluator_weights_->at("npoints_weight") * cur_validator_results.at(0);

  // Displacement

  cur_validator_results =
      cone_validators_->at("displacement")->coneValidator(&cluster, ground_plane);
  // index 0 = ratio between the x axis displacement and the minimum distance for that axis.
  // index 1 = ratio between the y axis displacement and the minimum distance for that axis.
  // index 2 = ratio between the z axis displacement and the minimum distance for that axis.

  confidence += evaluator_weights_->at("displacement_x_weight") * cur_validator_results.at(0) +
                evaluator_weights_->at("displacement_y_weight") * cur_validator_results.at(1) +
                evaluator_weights_->at("displacement_z_weight") * cur_validator_results.at(2);

  // Deviation
  cur_validator_results = cone_validators_->at("deviation")->coneValidator(&cluster, ground_plane);
  // index 0 = if not in xoy interval, ratio between the std deviation of the cluster and the
  // maximum or minimum deviation, whichever is closest to.
  // index 1 = if not z in interval, ratio between the std deviation of the cluster and the maximum
  // or minimum deviation, whichever is the closest to.

  confidence += evaluator_weights_->at("deviation_xoy_weight") * cur_validator_results.at(0) +
                evaluator_weights_->at("deviation_z_weight") * cur_validator_results.at(1);

  // Adjust for precision issues [0,1]
  confidence = std::max(0.0, std::min(confidence, 1.0));

  cluster.set_confidence(confidence);
  return confidence >= min_confidence_;
}