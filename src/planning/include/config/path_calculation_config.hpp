#ifndef SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_

#include "config/midpoint_generator_config.hpp"

struct PathCalculationConfig {
  /**
   * @brief Configuration for the Midpoint Generator class.
   */
  MidpointGeneratorConfig midpoint_generator_;

  /**
   * @brief Flag to enable/disable the sliding window for path calculation.
   */
  bool use_sliding_window_;

  /**
   * @brief Gain applied to the angle term in the cost function.
   */
  double angle_gain_;

  /**
   * @brief Gain applied to the distance term in the cost function.
   */
  double distance_gain_;

  /**
   * @brief Exponent applied to the angle term in the cost function.
   */
  double angle_exponent_;

  /**
   * @brief Exponent applied to the distance term in the cost function.
   */
  double distance_exponent_;

  /**
   * @brief Maximum allowed cost for a path.
   */
  double max_cost_;

  /**
   * @brief Minimum allowed distance between consecutive points in the generated path.
   */
  double minimum_point_distance_;

  /**
   * @brief Number of recent path points discarded and recalculated each iteration.
   */
  int lookback_points_;

  /**
   * @brief Maximum search depth when exploring candidate paths.
   */
  int search_depth_;

  /**
   * @brief Maximum number of points added in an iteration.
   */
  int max_points_;

  /**
   * @brief Number of iterations before triggering a full path regeneration.
   */
  int reset_interval_;

  /**
   * @brief Flag to enable/disable path reset logic.
   */
  bool use_reset_path_;

  /**
   * @brief Default constructor.
   */
  PathCalculationConfig()
      : midpoint_generator_(),
        use_sliding_window_(true),
        angle_gain_(20.0),
        distance_gain_(5.0),
        angle_exponent_(3.0),
        distance_exponent_(1.0),
        max_cost_(30.0),
        minimum_point_distance_(1.0),
        lookback_points_(20),
        search_depth_(2),
        max_points_(50),
        reset_interval_(10),
        use_reset_path_(false) {}

  /**
   * @brief Parameterized constructor.
   */
  PathCalculationConfig(
      const MidpointGeneratorConfig& mg_config,
      bool use_sliding_window,
      double angle_gain,
      double distance_gain,
      double angle_exponent,
      double distance_exponent,
      double max_cost,
      double minimum_point_distance,
      int lookback_points,
      int search_depth,
      int max_points,
      int reset_interval,
      bool use_reset_path)
      : midpoint_generator_(mg_config),
        use_sliding_window_(use_sliding_window),
        angle_gain_(angle_gain),
        distance_gain_(distance_gain),
        angle_exponent_(angle_exponent),
        distance_exponent_(distance_exponent),
        max_cost_(max_cost),
        minimum_point_distance_(minimum_point_distance),
        lookback_points_(lookback_points),
        search_depth_(search_depth),
        max_points_(max_points),
        reset_interval_(reset_interval),
        use_reset_path_(use_reset_path) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_