#pragma once

#include <cmath>

#include "common_lib/structures/path_point.hpp"
#include "config/velocity_config.hpp"

using PathPoint = common_lib::structures::PathPoint;

/**
 * @brief class that defines the path smoothing algorithm
 *
 */
class VelocityPlanning {
private:
  /**
   * @brief configuration of the smoothing algorithm
   *
   */
  VelocityPlanningConfig config_;

  /**
   * @brief function to calculate the radius of the circle that passes through 3 points
   *
   * @param point1 first point
   * @param point2 second point
   * @param point3 third point
   * @return radius of the circle
   */
  double find_circle_center(PathPoint &point1, PathPoint &point2, PathPoint &point3);

  /**
   * @brief function to limit the speed of the car according to the curvature of the lookahead path
   *
   * @param points path points
   * @param velocities velocities of the path points
   * @param brake_acelleration maximum braking acelleration
   */
  void speed_limiter(std::vector<PathPoint> &points, std::vector<double> &velocities);

  /**
   * @brief function to calculate the speed of the car according to the curvature of the path
   *
   * @param radiuses radiuses vector of the path points
   * @param velocities velocities vector of the path points
   */
  void point_speed(std::vector<double> &radiuses, std::vector<double> &velocities);

public:
  /**
   * @brief Construct a new default Path Smoothing object
   *
   */
  VelocityPlanning() = default;
  /**
   * @brief Construct a new Path Smoothing object with a given configuration
   *
   */
  explicit VelocityPlanning(VelocityPlanningConfig config) : config_(config) {}

  void set_velocity(std::vector<PathPoint> &final_path);

  void trackdrive_velocity(std::vector<PathPoint> &final_path);

  void stop(std::vector<PathPoint> &final_path) {
    int size = final_path.size();
    for (int i = 0; i < size/2; ++i) {
      final_path[i].ideal_velocity = 0.0;
    }
  }
};
