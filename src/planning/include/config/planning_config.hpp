#ifndef SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_

#include <string>

#include "path_calculation_config.hpp"
#include "skidpad_config.hpp"
#include "smoothing_config.hpp"
#include "velocity_config.hpp"

struct PlanningParameters {
  /*---------------------- Midpoint Generator (mg_) ----------------------*/
  double mg_minimum_cone_distance_;
  double mg_maximum_cone_distance_;
  double mg_sliding_window_radius_;

  /*---------------------- Path Calculation (pc_) ------------------------*/
  bool pc_use_sliding_window_;
  double pc_angle_gain_;
  double pc_distance_gain_;
  double pc_angle_exponent_;
  double pc_distance_exponent_;
  double pc_max_cost_;
  int pc_lookback_points_;
  int pc_search_depth_;
  int pc_max_points_;
  double pc_minimum_point_distance_;
  int pc_reset_interval_;
  bool pc_use_reset_path_;

  /*---------------------- Skidpad (skidpad_) ----------------------------*/
  int skidpad_minimum_cones_;
  double skidpad_tolerance_;

  /*---------------------- Path Smoothing (smoothing_) -------------------*/
  int smoothing_spline_precision_;
  int smoothing_spline_order_;
  float smoothing_spline_coeffs_ratio_;
  float smoothing_min_path_point_distance_;
  bool smoothing_use_path_smoothing_;
  bool smoothing_use_optimization_;
  double smoothing_car_width_;
  double smoothing_safety_margin_;
  double smoothing_curvature_weight_;
  double smoothing_smoothness_weight_;
  double smoothing_safety_weight_;
  int smoothing_max_iterations_;
  double smoothing_tolerance_;

  /*---------------------- Velocity Planning (vp_) -----------------------*/
  double vp_minimum_velocity_;
  double vp_braking_acceleration_;
  double vp_acceleration_;
  double vp_lateral_acceleration_;
  double vp_longitudinal_acceleration_;
  bool vp_use_velocity_planning_;
  double vp_desired_velocity_;

  /*---------------------- Planning (planning_) ----------------------*/
  /**
   * @brief Flag to enable/disable publishing of visualization messages.
   */
  bool planning_publishing_visualization_msgs_;

  /**
   * @brief Flag to enable/disable the use of simulated State Estimation.
   */
  bool planning_using_simulated_se_;

  /**
   * @brief Flag to enable/disable using planning with the full map
   */
  bool planning_using_full_map_;

  /**
   * @brief Distance to start braking during acceleration mode.
   */
  double planning_braking_distance_acceleration_;

  /**
   * @brief Distance to start braking during autocross/trackdrive mode.
   */
  double planning_braking_distance_autocross_;

  std::string map_frame_id_;
};

struct PlanningConfig {
  PathCalculationConfig path_calculation_;
  PathSmoothingConfig smoothing_;
  VelocityPlanningConfig velocity_planning_;
  SkidpadConfig skidpad_;
  bool publishing_visualization_msgs_;
  bool using_simulated_se_;
  bool using_full_map_;
  double braking_distance_acceleration_;
  double braking_distance_autocross_;

  PlanningConfig() = default;

  explicit PlanningConfig(const PlanningParameters &params)
      : path_calculation_(
            // Midpoint Generator Config
            MidpointGeneratorConfig(params.mg_minimum_cone_distance_,
                                    params.mg_maximum_cone_distance_,
                                    params.mg_sliding_window_radius_),
            // Path Calculation parameters
            params.pc_use_sliding_window_, params.pc_angle_gain_, params.pc_distance_gain_,
            params.pc_angle_exponent_, params.pc_distance_exponent_, params.pc_max_cost_,
            params.pc_minimum_point_distance_, params.pc_lookback_points_, params.pc_search_depth_,
            params.pc_max_points_, params.pc_reset_interval_, params.pc_use_reset_path_),
        smoothing_(params.smoothing_spline_precision_, params.smoothing_spline_order_,
                   params.smoothing_spline_coeffs_ratio_, params.smoothing_min_path_point_distance_,
                   params.smoothing_use_path_smoothing_, params.smoothing_use_optimization_,
                   params.smoothing_car_width_, params.smoothing_safety_margin_,
                   params.smoothing_curvature_weight_, params.smoothing_smoothness_weight_,
                   params.smoothing_safety_weight_, params.smoothing_max_iterations_,
                   params.smoothing_tolerance_),
        velocity_planning_(params.vp_minimum_velocity_, params.vp_desired_velocity_,
                           params.vp_braking_acceleration_, params.vp_acceleration_,
                           params.vp_lateral_acceleration_, params.vp_longitudinal_acceleration_,
                           params.vp_use_velocity_planning_),
        skidpad_(params.skidpad_minimum_cones_, params.skidpad_tolerance_),
        publishing_visualization_msgs_(params.planning_publishing_visualization_msgs_),
        using_simulated_se_(params.planning_using_simulated_se_),
        using_full_map_(params.planning_using_full_map_),
        braking_distance_acceleration_(params.planning_braking_distance_acceleration_),
        braking_distance_autocross_(params.planning_braking_distance_autocross_) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_