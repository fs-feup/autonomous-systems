#pragma once
#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>
#include <utils/evaluator_parameters.hpp>
#include <utils/ground_grid.hpp>

/**
 * @class ConeEvaluator
 *
 * @brief class that evaluates the cluster as a cone on a [0,1] confidence value
 *
 */
class ConeEvaluator {
private:
  std::shared_ptr<EvaluatorParameters> params_;

  bool close_to_ground(Cluster& cluster, const GroundGrid& ground_grid) const;

  bool cylinder_fits_cone(Cluster& cluster) const;

  bool npoints_valid(Cluster& cluster) const;

public:
  /**
   * @brief Constructs a new DeviationValidator object with specified intervals on the deviation.
   * @param params Struct with all validators, weights and minimum confidence.
   */
  ConeEvaluator(std::shared_ptr<EvaluatorParameters> params);

  /**
   * @brief Perform the cluster evaluation, changes the clusters confidence attribute to the
   * obtained result.
   *
   * @param cluster Cluster to evaluate.
   * @param ground_plane The plane against which some validators compare the cluster.
   * @return True if cluster is equal or above minimum confidence and false if below.
   */
  bool evaluateCluster(Cluster& cluster, const GroundGrid& ground_grid);
};
