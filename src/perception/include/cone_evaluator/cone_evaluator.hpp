#ifndef CONE_EVALUATOR_HPP
#define CONE_EVALUATOR_HPP

#include <utils/cluster.hpp>

class ConeEvaluator {
 public:
  virtual double evaluateCluster(Cluster& cluster) const = 0;
};

#endif  // CONE_EVALUATOR_HPP