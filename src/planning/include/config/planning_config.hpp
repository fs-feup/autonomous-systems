#ifndef SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_

#include "cone_coloring_config.hpp"
#include "outliers_config.hpp"
#include "path_calculation_config.hpp"
#include "simulation_config.hpp"
#include "smoothing_config.hpp"
#include "velocity_config.hpp"

struct PlanningParameters {
  double angle_gain_;
  double distance_gain_;
  double ncones_gain_;
  double angle_exponent_;
  double distance_exponent_;
  double cost_max_;
  double same_cone_distance_threshold_;
  bool use_memory_cone_coloring_;
  int outliers_spline_order_;
  float outliers_spline_coeffs_ratio_;
  int outliers_spline_precision_;
  double path_calculation_dist_threshold_;
  int smoothing_spline_order_;
  float smoothing_spline_coeffs_ratio_;
  int smoothing_spline_precision_;
  bool publishing_visualization_msgs_;
  bool using_simulated_se_;
  bool use_outlier_removal_;
  bool use_path_smoothing_;
  double desired_velocity_;
  double minimum_velocity_;
  double braking_acceleration_;
  double normal_acceleration_;
  bool use_velocity_planning_;
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
  VelocityPlanningConfig velocity_planning_;

  PlanningConfig() = default;
  explicit PlanningConfig(const PlanningParameters &params) {
    cone_coloring_.angle_weight_ = params.angle_gain_;
    cone_coloring_.distance_weight_ = params.distance_gain_;
    cone_coloring_.ncones_weight_ = params.ncones_gain_;
    cone_coloring_.angle_exponent_ = params.angle_exponent_;
    cone_coloring_.distance_exponent_ = params.distance_exponent_;
    cone_coloring_.max_cost_ = params.cost_max_;
    cone_coloring_.same_cone_distance_threshold_ = params.same_cone_distance_threshold_;
    cone_coloring_.use_memory_ = params.use_memory_cone_coloring_;

    outliers_.order_ = params.outliers_spline_order_;
    outliers_.precision_ = params.outliers_spline_precision_;
    outliers_.coeffs_ratio_ = params.outliers_spline_coeffs_ratio_;
    outliers_.use_outlier_removal_ = params.use_outlier_removal_;

    path_calculation_.dist_threshold_ = params.path_calculation_dist_threshold_;

    smoothing_.order_ = params.smoothing_spline_order_;
    smoothing_.precision_ = params.smoothing_spline_precision_;
    smoothing_.coeffs_ratio_ = params.smoothing_spline_coeffs_ratio_;
    smoothing_.use_path_smoothing_ = params.use_path_smoothing_;
    smoothing_.use_memory_ = params.use_memory_cone_coloring_;

    simulation_.publishing_visualization_msgs_ = params.publishing_visualization_msgs_;
    simulation_.using_simulated_se_ = params.using_simulated_se_;

    velocity_planning_.safety_speed_ = params.minimum_velocity_;
    velocity_planning_.braking_acceleration_ = params.braking_acceleration_;
    velocity_planning_.normal_acceleration_ = params.normal_acceleration_;
    velocity_planning_.use_velocity_planning_ = params.use_velocity_planning_;
  }
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_