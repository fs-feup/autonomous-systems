#ifndef SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct PathCalculationConfig {
  double dist_threshold_ = 7.0;

  bool use_cone_colouring_ = true;

  PathCalculationConfig() = default;
  explicit PathCalculationConfig(float dist_threshold, bool use_cone_colouring)
      : dist_threshold_(dist_threshold), use_cone_colouring_(use_cone_colouring) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_PATH_CALCULATION_CONFIG_HPP_