#pragma once
#include <cone_validator/cone_validator.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <utils/cluster.hpp>
#include <utils/evaluator_parameters.hpp>
#include <utils/ground_grid.hpp>

/**
 * @class ConeEvaluator
 *
 * @brief class that evaluates the cluster as a cone or not
 *
 */
class ConeEvaluator {
public:
  /**
   * @brief Constructs a new DeviationValidator object with specified intervals on the deviation.
   * @param params Struct with all validators, weights and minimum confidence.
   */
  ConeEvaluator(std::shared_ptr<EvaluatorParameters> params);

  /**
   * @brief Perform the cluster evaluation, guaranteeing they pass all validators
   *
   * @param cluster Cluster to evaluate.
   * @param ground_grid The ground grid to use for ground proximity checks.
   * @return True if cluster is a valid cone, false otherwise.
   */
  bool evaluateCluster(Cluster& cluster, const GroundGrid& ground_grid);

private:
  std::shared_ptr<EvaluatorParameters> params_;

  /**
   * @brief Check if the cluster is close to the ground
   *
   * @param cluster Cluster to evaluate
   * @param ground_grid Ground grid to use for ground height lookup
   */
  bool close_to_ground(Cluster& cluster, const GroundGrid& ground_grid) const;

  /**
   * @brief Check if a cylinder fits the cone dimensions
   * @param cluster Cluster to evaluate
   */
  bool cylinder_fits_cone(Cluster& cluster) const;

  /**
   * @brief Check if the number of points in the cluster is valid, according to the distance
   * @param cluster Cluster to evaluate
   */
  bool npoints_valid(Cluster& cluster) const;
};
