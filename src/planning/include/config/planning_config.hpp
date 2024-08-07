#ifndef SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_

#include "cone_coloring_config.hpp"
#include "outliers_config.hpp"
#include "path_calculation_config.hpp"
#include "simulation_config.hpp"
#include "smoothing_config.hpp"

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
  long double desired_velocity_;
  std::string map_frame_id_;
};

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct PlanningConfig {
  ConeColoringConfig cone_coloring_;
  OutliersConfig outliers_;
  PathCalculationConfig path_calculation_;
  PathSmoothingConfig smoothing_;
  SimulationConfig simulation_;

  PlanningConfig() = default;
  explicit PlanningConfig(const PlanningParameters &params) {
    cone_coloring_.angle_weight_ = params.angle_gain_;
    cone_coloring_.distance_weight_ = params.distance_gain_;
    cone_coloring_.ncones_weight_ = params.ncones_gain_;
    cone_coloring_.angle_exponent_ = params.angle_exponent_;
    cone_coloring_.distance_exponent_ = params.distance_exponent_;
    cone_coloring_.max_cost_ = params.cost_max_;

    outliers_.order_ = params.outliers_spline_order_;
    outliers_.precision_ = params.outliers_spline_precision_;
    outliers_.coeffs_ratio_ = params.outliers_spline_coeffs_ratio_;

    path_calculation_.dist_threshold_ = params.path_calculation_dist_threshold_;

    smoothing_.order_ = params.smoothing_spline_order_;
    smoothing_.precision_ = params.smoothing_spline_precision_;
    smoothing_.coeffs_ratio_ = params.smoothing_spline_coeffs_ratio_;

    simulation_.publishing_visualization_msgs_ = params.publishing_visualization_msgs_;
    simulation_.using_simulated_se_ = params.using_simulated_se_;
  }
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_