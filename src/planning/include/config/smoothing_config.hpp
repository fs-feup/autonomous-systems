#ifndef SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Path Smoothing class.
 *
 */
struct PathSmoothingConfig {
  /**
   * @brief Number of interpolated points between each pair of input points.
   *
   * Higher values produce a denser and smoother path at the cost of computation.
   */
  int spline_precision_ = 10;

  /**
   * @brief Order of the B-spline used for smoothing.
   *
   */
  int spline_order_ = 3;

  /**
   * @brief Ratio between number of spline coefficients and input points.
   *
   * Higher values produce a smoother but less flexible spline.
   */
  float spline_coeffs_ratio_ = 3.0f;

  /**
   * @brief Flag to enable/disable spline-based path smoothing.
   */
  bool use_path_smoothing_ = true;

  /**
   * @brief Flag to enable/disable optimization-based refinement after spline fitting.
   *
   */
  bool use_optimization_ = true;

  /**
   * @brief Vehicle width in meters.
   *
   */
  double car_width_ = 1.2;

  /**
   * @brief Additional safety clearance in meters.
   *
   */
  double safety_margin_ = 0.3;

  /**
   * @brief Weight for curvature minimization in the optimization cost function.
   */
  double curvature_weight_ = 100.0;

  /**
   * @brief Weight for path smoothness in the optimization cost function.
   */
  double smoothness_weight_ = 0.1;

  /**
   * @brief Weight for safety distance from obstacles in the optimization cost function.
   */
  double safety_weight_ = 100'000.0;

  /**
   * @brief Maximum number of iterations for the optimization solver.
   */
  int max_iterations_ = 2000;

  /**
   * @brief Convergence tolerance for the optimization solver.
   */
  double tolerance_ = 1e-4;

  PathSmoothingConfig() = default;
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SMOOTHING_CONFIG_HPP_