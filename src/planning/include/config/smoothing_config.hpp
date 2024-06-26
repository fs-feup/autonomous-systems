#ifndef SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_

/**
 * @brief struct for the configuration of the smoothing algorithm
 *
 */
struct PathSmoothingConfig {
  /**
   * @brief ratio of output to input cones
   *
   */
  int precision_ = 10;
  /**
   * @brief order of the spline
   *
   */
  int order_ = 3;
  /**
   * @brief ratio of coefficients to input points
   *
   */
  float coeffs_ratio_ = 3.0;
  PathSmoothingConfig() = default;
  PathSmoothingConfig(int precision, int order, float coeffs_ratio)
      : precision_(precision), order_(order), coeffs_ratio_(coeffs_ratio) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_