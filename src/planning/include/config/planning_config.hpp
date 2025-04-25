#ifndef SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_

#include "outliers_config.hpp"
#include "path_calculation_config.hpp"
#include "simulation_config.hpp"
#include "smoothing_config.hpp"
#include "velocity_config.hpp"
#include <string>

struct PlanningParameters {
  double minimum_cone_distance_;

  double projected_point_distance_;
  double nc_angle_gain_;
  double nc_distance_gain_;
  double nc_angle_exponent_;
  double nc_distance_exponent_;
  double nc_max_cost_;
  int nc_search_depth_;
  int nc_max_points_;
  bool use_outlier_removal_;
  int smoothing_spline_order_;
  float smoothing_spline_coeffs_ratio_;
  int smoothing_spline_precision_;
  bool use_path_smoothing_;
  bool publishing_visualization_msgs_;
  bool using_simulated_se_;

  int outliers_spline_order_;
  float outliers_spline_coeffs_ratio_;
  int outliers_spline_precision_;

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
  OutliersConfig outliers_;
  PathCalculationConfig path_calculation_;
  PathSmoothingConfig smoothing_;
  SimulationConfig simulation_;
  VelocityPlanningConfig velocity_planning_;

  PlanningConfig() = default;
  explicit PlanningConfig(const PlanningParameters &params) {
    outliers_.order_ = params.outliers_spline_order_;
    outliers_.precision_ = params.outliers_spline_precision_;
    outliers_.coeffs_ratio_ = params.outliers_spline_coeffs_ratio_;
    outliers_.use_outlier_removal_ = params.use_outlier_removal_;

<<<<<<< HEAD
    path_calculation_.dist_threshold_ = params.path_calculation_dist_threshold_;
=======
    path_calculation_.minimum_cone_distance_ = params.minimum_cone_distance_;
    path_calculation_.projected_point_distance_ = params.projected_point_distance_;
>>>>>>> dev
    path_calculation_.angle_gain_ = params.nc_angle_gain_;
    path_calculation_.distance_gain_ = params.nc_distance_gain_;
    path_calculation_.angle_exponent_ = params.nc_angle_exponent_;
    path_calculation_.distance_exponent_ = params.nc_distance_exponent_;
    path_calculation_.max_cost_ = params.nc_max_cost_;
    path_calculation_.search_depth_ = params.nc_search_depth_;
    path_calculation_.max_points_ = params.nc_max_points_;


    smoothing_.order_ = params.smoothing_spline_order_;
    smoothing_.precision_ = params.smoothing_spline_precision_;
    smoothing_.coeffs_ratio_ = params.smoothing_spline_coeffs_ratio_;
    smoothing_.use_path_smoothing_ = params.use_path_smoothing_;

    simulation_.publishing_visualization_msgs_ = params.publishing_visualization_msgs_;
    simulation_.using_simulated_se_ = params.using_simulated_se_;

    velocity_planning_.minimum_velocity_ = params.minimum_velocity_;
    velocity_planning_.braking_acceleration_ = params.braking_acceleration_;
    velocity_planning_.normal_acceleration_ = params.normal_acceleration_;
    velocity_planning_.use_velocity_planning_ = params.use_velocity_planning_;
  }
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_