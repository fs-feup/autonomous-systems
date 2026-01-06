#ifndef SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_

#include <osqp.h>

#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <vector>

#include "common_lib/structures/path_point.hpp"
#include "config/smoothing_config.hpp"
#include "utils/splines.hpp"

using PathPoint = common_lib::structures::PathPoint;

/**
 * @brief class that defines the path smoothing algorithm
 *
 */
class PathSmoothing {
public:
  /**
   * @brief Construct a new default Path Smoothing object
   *
   */
  PathSmoothing() = default;
  
  /**
   * @brief Construct a new Path Smoothing object with a given configuration
   *
   */
  explicit PathSmoothing(PathSmoothingConfig config) : config_(config) {}

  /**
   * @brief Smooths a path by fitting a B-spline through the input points. This function provides
   * a simple interface for path smoothing without boundary constraints or optimization.
   *
   * @param path The input path to be smoothed
   * @return std::vector<PathPoint> The smoothed path
   *
   */
  std::vector<PathPoint> smooth_path(std::vector<PathPoint>& path) const;

  /**
   * @brief Optimizes a racing line path by fitting splines through track boundaries and applying
   * quadratic programming optimization.
   *
   * @param path The initial center path to be optimized
   * @param yellow_cones Track boundary markers on the right boundary
   * @param blue_cones Track boundary markers on the left boundary
   *
   * @return std::vector<PathPoint> The optimized path
   *
   */
  std::vector<PathPoint> optimize_path(std::vector<PathPoint>& path,
                                       std::vector<PathPoint>& yellow_cones,
                                       std::vector<PathPoint>& blue_cones) const;

private:
  /**
   * @brief configuration of the smoothing algorithm
   *
   */
  PathSmoothingConfig config_;

  /**
   * @brief Optimizes a path using quadratic programming (OSQP) to balance smoothness, curvature,
   * and safety constraints. The function takes a center line and left/right boundaries, then
   * computes an optimized path that minimizes curvature and jerk while staying within the track
   * boundaries with a safety margin.
   *
   * @param center Sequence of points representing the initial center line path
   * @param left Sequence of points representing the left track boundary
   * @param right Sequence of points representing the right track boundary
   * @return std::vector<PathPoint> Optimized path
   *
   */
  std::vector<PathPoint> osqp_optimization(const std::vector<PathPoint>& center,
                                           const std::vector<PathPoint>& left,
                                           const std::vector<PathPoint>& right) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_