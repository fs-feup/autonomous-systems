#ifndef SRC_PLANNING_INCLUDE_UTILS_OUTLIERS_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_UTILS_OUTLIERS_CONFIG_HPP_

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct OutliersConfig {
  /**
   * @brief ratio of output cones to input cones
   *
   */
  int precision = 1;
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
  OutliersConfig() = default;
  OutliersConfig(int precision, int order, float coeffs_ratio)
      : precision(precision), order(order), coeffs_ratio(coeffs_ratio) {}
};

#endif  // SRC_PLANNING_INCLUDE_UTILS_OUTLIERS_CONFIG_HPP_