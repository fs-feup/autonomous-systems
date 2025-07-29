#ifndef SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct PathCalculationConfig {
  double minimum_cone_distance_ = 2.5;
  double maximum_cone_distance_ = 10.0;
  double dist_threshold_ = 7.0;
  double angle_gain_ = 20.0;
  double distance_gain_ = 5.0;
  double angle_exponent_ = 3.0;
  double distance_exponent_ = 1.0;
  double tolerance_ = 1.0;
  double skidpad_tolerance_ = 1.0;
  double max_cost_ = 30.0;
  int lookback_points_ = 20;
  int search_depth_ = 2;
  int max_points_ = 50;
  int reset_global_path_ = 10;
  int skidpad_minimum_cones_ = 10.0;

  PathCalculationConfig() = default;
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_