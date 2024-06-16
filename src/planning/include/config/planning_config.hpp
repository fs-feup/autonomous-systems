#ifndef SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_

#include "cone_coloring_config.hpp"
#include "outliers_config.hpp"
#include "path_calculation_config.hpp"
#include "smoothing_config.hpp"
#include "simulation_config.hpp"

struct PlanningParameters {
  double angle_gain_;
  double distance_gain_;
  double ncones_gain_;
  double angle_exponent_;
  double distance_exponent_;
  double cost_max_;
  int outliers_spline_order_;
  float outliers_spline_coeffs_ratio_;
  int outliers_spline_precision_;
  double path_calculation_dist_threshold_;
  int smoothing_spline_order_;
  float smoothing_spline_coeffs_ratio_;
  int smoothing_spline_precision_;
  bool publishing_visualization_msgs_;
  bool using_simulated_se_;
};

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct PlanningConfig {
    ConeColoringConfig cone_coloring;
    OutliersConfig outliers;
    PathCalculationConfig path_calculation;
    PathSmoothingConfig smoothing;
    SimulationConfig simulation;

    PlanningConfig() = default;
    PlanningConfig(const PlanningParameters &params) {
        cone_coloring.angle_weight = params.angle_gain_;
        cone_coloring.distance_weight = params.distance_gain_;
        cone_coloring.ncones_weight = params.ncones_gain_;
        cone_coloring.angle_exponent = params.angle_exponent_;
        cone_coloring.distance_exponent = params.distance_exponent_;
        cone_coloring.max_cost = params.cost_max_;

        outliers.order = params.outliers_spline_order_;
        outliers.precision = params.outliers_spline_coeffs_ratio_;
        outliers.coeffs_ratio = params.outliers_spline_precision_;

        path_calculation.dist_threshold = params.path_calculation_dist_threshold_;

        smoothing.order = params.smoothing_spline_order_;
        smoothing.precision = params.smoothing_spline_coeffs_ratio_;
        smoothing.coeffs_ratio = params.smoothing_spline_precision_;

        simulation.publishing_visualization_msgs = params.publishing_visualization_msgs_;
        simulation.using_simulated_se = params.using_simulated_se_;
    }
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_