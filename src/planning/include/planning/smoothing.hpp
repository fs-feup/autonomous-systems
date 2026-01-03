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

  std::vector<PathPoint> smooth_path(std::vector<PathPoint>& path) const;

  std::vector<PathPoint> optimize_path(std::vector<PathPoint>& path,
                                       std::vector<PathPoint>& yellow_cones,
                                       std::vector<PathPoint>& blue_cones) const;

private:
  /**
   * @brief configuration of the smoothing algorithm
   *
   */
  PathSmoothingConfig config_;

  std::vector<PathPoint> osqp_optimization(const std::vector<PathPoint>& center,
                                           const std::vector<PathPoint>& left,
                                           const std::vector<PathPoint>& right) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_