#pragma once

#include <cmath>

#include "common_lib/structures/path_point.hpp"
#include "config/velocity_config.hpp"

using PathPoint = common_lib::structures::PathPoint;

/**
 * @brief Computes velocity profiles for a planned path based on curvature and dynamics constraints.
 *
 * The VelocityPlanning class generates a velocity profile along a path by:
 * - Estimating curvature (via circle fitting between consecutive path points),
 * - Deriving maximum allowable velocities from lateral acceleration limits,
 * - Propagating braking constraints backward along the path.
 */
class VelocityPlanning {
public:
  /**
   * @brief Construct a new default Velocity Planning object
   *
   */
  VelocityPlanning() = default;
  /**
   * @brief Construct a new Velocity Planning object with a given configuration
   *
   */
  explicit VelocityPlanning(VelocityPlanningConfig config) : config_(config) {}

  /**
   * @brief Assigns an ideal velocity to each point of the path.
   *
   * @param final_path Vector of path points to update with planned velocities.
   */
  void set_velocity(std::vector<PathPoint> &final_path);

  /**
   * @brief Computes velocity for track driving scenarios with repeated smoothing.
   *
   * @param final_path Vector of path points to update with planned velocities.
   */
  void trackdrive_velocity(std::vector<PathPoint> &final_path);

  /**
   * @brief Gradually reduces velocity to zero along the first half of the path.
   *
   * Used for controlled stopping. It accumulates distance along the path and sets the
   * velocity to zero once a specified distance threshold is reached.
   *
   * @param final_path Vector of path points to update for stopping behavior.
   */
  void stop(std::vector<PathPoint> &final_path);

private:
  /**
   * @brief configuration of the velocity planning algorithm
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
  double find_circle_center(const PathPoint &point1, const PathPoint &point2,
                            const PathPoint &point3);

  /**
   * @brief Applies a forward acceleration constraint to the velocity profile.
   *
   *
   * @param points       The sequence of path points containing positions.
   * @param velocities   The velocity vector to be modified in-place.
   */
  void accelaration_limiter(const std::vector<PathPoint> &points,
                                              std::vector<double> &velocities);

  /**
   * @brief function to limit the speed of the car according to braking constraints
   *
   * @param points path points
   * @param velocities velocities of the path points
   */
  void braking_limiter(std::vector<PathPoint> &points, std::vector<double> &velocities);

  /**
   * @brief function to calculate the speed of the car according to the curvature of the path
   *
   * @param radiuses radiuses vector of the path points
   * @param velocities velocities vector of the path points
   */
  void point_speed(const std::vector<double> &radiuses, std::vector<double> &velocities);
};