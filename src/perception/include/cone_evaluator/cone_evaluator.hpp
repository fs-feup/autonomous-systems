#pragma once
#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>

/**
 * @class ConeEvaluator
 *
 * @brief class that evaluates the cluster as a cone on a [0,1] confidence value
 *
 */
class ConeEvaluator {
private:
  std::unordered_map<std::string, ConeValidator> cone_validators_;
  std::unordered_map<std::string, double> validator_weights_;

public:
  /**
   * @brief Constructs a new DeviationValidator object with specified intervals on the deviation.
   * @param cone_validators Minimum xOy plane deviation.
   * @param validator_weights Maximum xOy plane deviation.
   */
  ConeEvaluator(std::unordered_map<std::string, ConeValidator> cone_validators,
                std::unordered_map<std::string, double> validator_weights);

  /**
   * @brief Perform the cluster evaluation, changes the clusters confidence attribute to the
   * obtained result.
   *
   * @param cluster Cluster to evaluate
   */
  void evaluateCluster(Cluster& cluster) const = 0;
};
