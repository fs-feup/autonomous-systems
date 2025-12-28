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
  /**
   * @brief flag to enable/disable the path smoothing
   *
   */
  bool use_path_smoothing_ = true;
  /**
   * @brief Enable OSQP optimization after spline
   *
   */
  bool use_optimization_;
  /**
   * @brief Vehicle width in meters
   *
   */
  double car_width_;
  /**
   * @brief Additional safety clearance in meters
   *
   */
  double safety_margin_;
  /**
   * @brief Weight for curvature minimization
   *
   */
  double curvature_weight_;
  /**
   * @brief  Weight for path smoothness
   *
   */
  double smoothness_weight_;
  /**
   * @brief Weight to stay close to spline path
   *
   */
  double deviation_weight_;
  /**
   * @brief OSQP max iterations
   *
   */
  int max_iterations_;
  /**
   * @brief  OSQP convergence tolerance
   *
   */
  double tolerance_;
  PathSmoothingConfig() = default;
  PathSmoothingConfig(int precision, int order, float coeffs_ratio, bool use_path_smoothing,
                      bool use_optimization, double car_width, double safety_margin,
                      double curvature_weight, double smoothness_weight, double deviation_weight,
                      int max_iterations, double tolerance)
      : precision_(precision),
        order_(order),
        coeffs_ratio_(coeffs_ratio),
        use_path_smoothing_(use_path_smoothing),
        use_optimization_(use_optimization),
        car_width_(car_width),
        safety_margin_(safety_margin),
        curvature_weight_(curvature_weight),
        smoothness_weight_(smoothness_weight),
        deviation_weight_(deviation_weight),
        max_iterations_(max_iterations),
        tolerance_(tolerance) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_