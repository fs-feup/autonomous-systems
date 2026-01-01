#ifndef SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_

#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "config/smoothing_config.hpp"
#include "utils/splines.hpp"

constexpr double MAX_DISTANCE_BETWEEN_POINTS = 4.5;

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
   * @brief function to smooth a path
   *
   * @param unordered_path input vector of path points
   * @return std::vector<PathPoint> smoothed path points
   */
  void smooth_path(std::vector<PathPoint>& path, std::vector<PathPoint>& yellow_cones,
                   std::vector<PathPoint>& blue_cones) const;

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
   * @param initial_car_orientation initial orientation of the car (usually 0 but not on some tests)
   */
  // void order_path(std::vector<PathPoint>& unord_path, const Pose& car_pose,
  //                 const double initial_car_orientation) const;

  std::vector<PathPoint> optimize_centerline_osqp(const std::vector<PathPoint>& center,
                                                  const std::vector<PathPoint>& left,
                                                  const std::vector<PathPoint>& right) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_