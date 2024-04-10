#ifndef DISTANCE_PREDICT_HPP
#define DISTANCE_PREDICT_HPP

#include <utils/cluster.hpp>
#include <cone_evaluator/cone_evaluator.hpp>

class DistancePredict : public ConeEvaluator {
 private:
  double vertical_ang_res;
  double horizontal_ang_res;
 public:
  DistancePredict(double vertical_ang_res, double horizontal_ang_res);
  double evaluateCluster(Cluster& cluster) const override;
};

#endif  // DISTANCE_PREDICT_HPP