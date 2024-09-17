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

  /**
   * @brief flag to use cone colouring
   *
   */
  bool use_cone_colouring_ = true;
  /**
   * @brief flag to enable/disable the outliers removal
   *
   */
  bool use_outlier_removal_ = true;
  OutliersConfig() = default;
  OutliersConfig(int precision, int order, float coeffs_ratio, bool use_cone_coloring,
                 bool use_outlier_removal)
      : precision_(precision),
        order_(order),
        coeffs_ratio_(coeffs_ratio),
        use_cone_colouring_(use_cone_coloring),
        use_outlier_removal_(use_outlier_removal) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_OUTLIERS_CONFIG_HPP_