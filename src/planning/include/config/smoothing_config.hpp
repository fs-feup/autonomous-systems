#ifndef SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Path Smoothing class.
 */
struct PathSmoothingConfig {
  /**
   * @brief Number of interpolated points between each pair of input points.
   */
  int spline_precision_;

  /**
   * @brief Order of the B-spline used for smoothing.
   */
  int spline_order_;

  /**
   * @brief Ratio between number of spline coefficients and input points.
   */
  float spline_coeffs_ratio_;

  /**
   * @brief Minimum distance between consecutive path points after smoothing.
   */
  float min_path_point_distance_;

  /**
   * @brief Flag to enable/disable spline-based path smoothing.
   */
  bool use_path_smoothing_;

  /**
   * @brief Flag to enable/disable optimization-based refinement after spline fitting.
   */
  bool use_optimization_;

  /**
   * @brief Vehicle width in meters.
   */
  double car_width_;

  /**
   * @brief Additional safety clearance in meters.
   */
  double safety_margin_;

  /**
   * @brief Weight for curvature minimization in the optimization cost function.
   */
  double curvature_weight_;

  /**
   * @brief Weight for path smoothness in the optimization cost function.
   */
  double smoothness_weight_;

  /**
   * @brief Weight for safety distance from obstacles in the optimization cost function.
   */
  double safety_weight_;

  /**
   * @brief Maximum number of iterations for the optimization solver.
   */
  int max_iterations_;

  /**
   * @brief Convergence tolerance for the optimization solver.
   */
  double tolerance_;

  /**
   * @brief Default constructor.
   */
  PathSmoothingConfig()
      : spline_precision_(10),
        spline_order_(3),
        spline_coeffs_ratio_(3.0f),
        min_path_point_distance_(0.3f),
        use_path_smoothing_(true),
        use_optimization_(true),
        car_width_(1.2),
        safety_margin_(0.3),
        curvature_weight_(100.0),
        smoothness_weight_(0.1),
        safety_weight_(100'000.0),
        max_iterations_(2000),
        tolerance_(1e-4) {}

  /**
   * @brief Parameterized constructor.
   */
  PathSmoothingConfig(int spline_precision, int spline_order, float spline_coeffs_ratio,
                      float min_path_point_distance, bool use_path_smoothing, bool use_optimization,
                      double car_width, double safety_margin, double curvature_weight,
                      double smoothness_weight, double safety_weight, int max_iterations,
                      double tolerance)
      : spline_precision_(spline_precision),
        spline_order_(spline_order),
        spline_coeffs_ratio_(spline_coeffs_ratio),
        min_path_point_distance_(min_path_point_distance),
        use_path_smoothing_(use_path_smoothing),
        use_optimization_(use_optimization),
        car_width_(car_width),
        safety_margin_(safety_margin),
        curvature_weight_(curvature_weight),
        smoothness_weight_(smoothness_weight),
        safety_weight_(safety_weight),
        max_iterations_(max_iterations),
        tolerance_(tolerance) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_