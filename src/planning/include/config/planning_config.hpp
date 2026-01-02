#ifndef SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_

#include "path_calculation_config.hpp"
#include "simulation_config.hpp"
#include "smoothing_config.hpp"
#include "velocity_config.hpp"
#include "skidpad_config.hpp"
#include <string>

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
  double pc_tolerance_;
  int pc_reset_path_;
  bool pc_use_reset_path_;

  /*---------------------- Skidpad (skidpad_) ----------------------------*/
  int skidpad_minimum_cones_;
  double skidpad_tolerance_;

  /*---------------------- Path Smoothing (smoothing_) -------------------*/
  int smoothing_spline_order_;
  float smoothing_coeffs_ratio_;
  int smoothing_spline_precision_;
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
  double vp_normal_acceleration_;
  bool vp_use_velocity_planning_;
  double vp_desired_velocity_;

  /*---------------------- Simulation (simulation_) ----------------------*/
  bool simulation_publishing_visualization_msgs_;
  bool simulation_using_simulated_se_;

  std::string map_frame_id_;
};

struct PlanningConfig {
  PathCalculationConfig path_calculation_;
  PathSmoothingConfig smoothing_;
  SimulationConfig simulation_;
  VelocityPlanningConfig velocity_planning_;
  SkidpadConfig skidpad_;

  PlanningConfig() = default;

  explicit PlanningConfig(const PlanningParameters &params) {
    /*---------------------- Path Calculation (pc_) ------------------------*/

    path_calculation_.angle_gain_ = params.pc_angle_gain_;
    path_calculation_.distance_gain_ = params.pc_distance_gain_;
    path_calculation_.angle_exponent_ = params.pc_angle_exponent_;
    path_calculation_.distance_exponent_ = params.pc_distance_exponent_;
    path_calculation_.max_cost_ = params.pc_max_cost_;
    path_calculation_.lookback_points_ = params.pc_lookback_points_;
    path_calculation_.search_depth_ = params.pc_search_depth_;
    path_calculation_.max_points_ = params.pc_max_points_;
    path_calculation_.tolerance_ = params.pc_tolerance_;
    path_calculation_.reset_path_ = params.pc_reset_path_;
    path_calculation_.use_reset_path_ = params.pc_use_reset_path_;
    path_calculation_.use_sliding_window_ = params.pc_use_sliding_window_;

    /*---------------------- Midpoint Generator (mg_) ----------------------*/
    path_calculation_.midpoint_generator_.minimum_cone_distance_ =
        params.mg_minimum_cone_distance_;
    path_calculation_.midpoint_generator_.maximum_cone_distance_ =
        params.mg_maximum_cone_distance_;
    path_calculation_.midpoint_generator_.sliding_window_radius_ =
        params.mg_sliding_window_radius_;

    /*---------------------- Skidpad (skidpad_) ----------------------------*/
    skidpad_.minimum_cones_ = params.skidpad_minimum_cones_;
    skidpad_.tolerance_ = params.skidpad_tolerance_;

    /*---------------------- Path Smoothing (smoothing_) -------------------*/
    smoothing_.order_ = params.smoothing_spline_order_;
    smoothing_.precision_ = params.smoothing_spline_precision_;
    smoothing_.coeffs_ratio_ = params.smoothing_coeffs_ratio_;
    smoothing_.use_path_smoothing_ = params.smoothing_use_path_smoothing_;
    smoothing_.use_optimization_ = params.smoothing_use_optimization_;
    smoothing_.car_width_ = params.smoothing_car_width_;
    smoothing_.safety_margin_ = params.smoothing_safety_margin_;
    smoothing_.curvature_weight_ = params.smoothing_curvature_weight_;
    smoothing_.smoothness_weight_ = params.smoothing_smoothness_weight_;
    smoothing_.safety_weight_ = params.smoothing_safety_weight_;
    smoothing_.max_iterations_ = params.smoothing_max_iterations_;
    smoothing_.tolerance_ = params.smoothing_tolerance_;

    /*---------------------- Velocity Planning (vp_) -----------------------*/
    velocity_planning_.minimum_velocity_ = params.vp_minimum_velocity_;
    velocity_planning_.desired_velocity_ = params.vp_desired_velocity_;
    velocity_planning_.braking_acceleration_ = params.vp_braking_acceleration_;
    velocity_planning_.acceleration_ = params.vp_acceleration_;
    velocity_planning_.normal_acceleration_ = params.vp_normal_acceleration_;
    velocity_planning_.use_velocity_planning_ = params.vp_use_velocity_planning_;

    /*---------------------- Simulation (simulation_) ----------------------*/
    simulation_.publishing_visualization_msgs_ = params.simulation_publishing_visualization_msgs_;
    simulation_.using_simulated_se_ = params.simulation_using_simulated_se_;
  }
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PLANNING_CONFIG_HPP_
