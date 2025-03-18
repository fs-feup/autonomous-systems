#ifndef SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct PathCalculationConfig {
  double dist_threshold_ = 7.0;
  double angle_gain_ = 20.0;
  double distance_gain_ = 5.0;
  double angle_exponent_ = 3.0;
  double distance_exponent_ = 1.0;
  double max_cost_ = 30.0;
  int search_depth_ = 2;
  int max_points_ = 50;

  PathCalculationConfig() = default;
  explicit PathCalculationConfig(float dist_threshold) : dist_threshold_(dist_threshold) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_