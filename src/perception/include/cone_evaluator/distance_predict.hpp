#pragma once

#include <cone_evaluator/cone_evaluator.hpp>
#include <utils/cluster.hpp>

/**
 * @class DistancePredict
 *
 * @brief Cone heurisitic evaluation based on distance
 *
 */
class DistancePredict : public ConeEvaluator {
 private:
  double vertical_ang_res;    ///< LiDAR's vertical angular resolution (degrees)
  double horizontal_ang_res;  ///< LiDAR's horizontal angular resolution (degrees)
 public:
  /**
   * @brief Constructor of a Distance Predict
   *
   * @param vertical_ang_res LiDAR's vertical angular resolution (degrees)
   * @param horizontal_ang_res LiDAR's horizontal angular resolution (degrees)
   */
  DistancePredict(double vertical_ang_res, double horizontal_ang_res);

  /**
   * @brief Perform the cluster evaluation based on distance
   *
   * @param cluster Cluster to evaluate
   * @return double Cluster's confidence &isin; [0,1]
   */
  double evaluateCluster(Cluster& cluster) const override;
};
