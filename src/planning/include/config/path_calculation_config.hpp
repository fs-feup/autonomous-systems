#ifndef SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_

#include "config/midpoint_generator_config.hpp"

struct PathCalculationConfig {
  /**
   * @brief Configuration for the Midpoint Generator class.
   *
   */
  MidpointGeneratorConfig midpoint_generator_;

  /**
   * @brief Flag to enable/disable the sliding window for path calculation.
   *
   */
  bool use_sliding_window_ = true;

  /**
   * @brief Gain applied to the angle term in the cost function.
   *
   * Higher values make angle deviations more penalized when evaluating paths.
   */
  double angle_gain_ = 20.0;

  /**
   * @brief Gain applied to the distance term in the cost function.
   *
   * Higher values make distance deviations more penalized.
   */
  double distance_gain_ = 5.0;

  /**
   * @brief Exponent applied to the angle term in the cost function.
   *
   */
  double angle_exponent_ = 3.0;

  /**
   * @brief Exponent applied to the distance term in the cost function.
   *
   */
  double distance_exponent_ = 1.0;

  /**
   * @brief Maximum allowed cost for a path.
   *
   * Paths with higher cost than this value will be rejected.
   */
  double max_cost_ = 30.0;

  /**
   * @brief Minimum allowed distance between consecutive points in the generated path.
   *
   */
  double minimum_point_distance_ = 1.0;

  /**
   * @brief Number of recent path points discarded and recalculated each iteration.
   *
   */
  int lookback_points_ = 20;

  /**
   * @brief Maximum search depth when exploring candidate paths.
   *
   */
  int search_depth_ = 2;

  /**
   * @brief Maximum number of points added in an iteration
   *
   */
  int max_points_ = 50;

  /**
   * @brief Number of iterations before triggering a full path regeneration.
   *
   */
  int reset_interval_ = 10;

  /**
   * @brief Flag to enable/disable path reset logic.
   *
   */
  bool use_reset_path_ = false;

  PathCalculationConfig() = default;
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_