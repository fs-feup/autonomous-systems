#pragma once
#include <utils/cluster.hpp>

/**
 * @class ConeEvaluator
 *
 * @brief Abstract class that represents an heurisic that evaluates the cluster as a cone
 *
 */
class ConeEvaluator {
 public:
  /**
   * @brief Perform the cluster evaluation
   *
   * @param cluster Cluster to evaluate
   * @return double Cluster's confidence &isin; [0,1]
   */
  virtual double evaluate_cluster(Cluster& cluster) const = 0;
};
