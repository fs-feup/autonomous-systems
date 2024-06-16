#ifndef SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct PathCalculationConfig {
  double dist_threshold = 7.0;

  PathCalculationConfig() = default;
  PathCalculationConfig(float dist_threshold)
      : dist_threshold(dist_threshold) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_