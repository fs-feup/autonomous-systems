#ifndef SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_

#include <cmath>

#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "config/smoothing_config.hpp"
#include "utils/splines.hpp"

constexpr double MAX_DISTANCE_BETWEEN_POINTS = 8.5;

using PathPoint = common_lib::structures::PathPoint;
using Pose = common_lib::structures::Pose;

/**
 * @brief class that defines the path smoothing algorithm
 *
 */
class PathSmoothing {
private:
  /**
   * @brief configuration of the smoothing algorithm
   *
   */
  PathSmoothingConfig config_;
  /**
   * @brief function to order the path points to be used in the spline fitting
   *
   * @param unord_path unoredred path points
   * @param car_pose pose of the car to start ordering according to the closest point
   */
  void order_path(std::vector<PathPoint>& unord_path, const Pose& car_pose) const;

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
   * @brief function to smooth a path
   *
   * @param unordered_path input vector of unordered path points
   * @param car_pose pose of the car to start ordering according to the closest point
   * @return std::vector<PathPoint> smoothed path points ordered from the closest to the car to
   * farthest
   */
  std::vector<PathPoint> smooth_path(std::vector<PathPoint>& unordered_path,
                                     const Pose& car_pose) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_