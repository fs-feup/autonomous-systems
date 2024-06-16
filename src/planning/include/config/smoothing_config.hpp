#ifndef SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_

/**
 * @brief struct for the configuration of the smoothing algorithm
 *
 */
struct SmoothingConfig {
  /**
   * @brief ratio of output to input cones
   *
   */
  int precision = 10;
  /**
   * @brief order of the spline
   *
   */
  int order = 3;
  /**
   * @brief ratio of coefficients to input points
   *
   */
  float coeffs_ratio = 3.0;
  SmoothingConfig() = default;
  SmoothingConfig(int precision, int order, float coeffs_ratio)
      : precision(precision), order(order), coeffs_ratio(coeffs_ratio) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_