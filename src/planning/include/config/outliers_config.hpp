#ifndef SRC_PLANNING_INCLUDE_CONFIG_OUTLIERS_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_OUTLIERS_CONFIG_HPP_

/**
 * @brief struct for the configuration of the outliers removal algorithm.
 *
 */
struct OutliersConfig {
  /**
   * @brief ratio of output cones to input cones
   *
   */
  int precision_ = 1;
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
  OutliersConfig() = default;
  OutliersConfig(int precision, int order, float coeffs_ratio)
      : precision_(precision), order_(order), coeffs_ratio_(coeffs_ratio) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_OUTLIERS_CONFIG_HPP_