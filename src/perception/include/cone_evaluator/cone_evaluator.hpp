#pragma once
#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/**
 * @class ConeEvaluator
 *
 * @brief class that evaluates the cluster as a cone on a [0,1] confidence value
 *
 */
class ConeEvaluator {
private:
  std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<ConeValidator>>> cone_validators_;
  std::shared_ptr<std::unordered_map<std::string, double>> evaluator_weights_;
  double min_confidence_;

public:
  /**
   * @brief Constructs a new DeviationValidator object with specified intervals on the deviation.
   * @param cone_validators Map with all cone validators and their names.
   * @param evaluator_weights Map with all weights.
   * @param min_confidence Minimum confidence needed for the cluster to be accepted.
   */
  ConeEvaluator(std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<ConeValidator>>>
                    cone_validators,
                std::shared_ptr<std::unordered_map<std::string, double>> evaluator_weights,
                double min_confidence);

  /**
   * @brief Perform the cluster evaluation, changes the clusters confidence attribute to the
   * obtained result.
   *
   * @param cluster Cluster to evaluate.
   * @param ground_plane The plane against which some validators compare the cluster.
   * @return True if cluster is equal or above minimum confidence and false if below.
   */
  bool evaluateCluster(Cluster& cluster, Plane& ground_plane);
};
