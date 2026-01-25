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

  static constexpr double epsilon = 1e-9;
  static constexpr double gravity = 9.81;

  double find_curvature(const PathPoint &p1, const PathPoint &p2, const PathPoint &p3);

  void point_speed(const std::vector<double> &radiuses, std::vector<double> &velocities);

  void acceleration_limiter(const std::vector<PathPoint> &points, std::vector<double> &velocities,
                            const std::vector<double> &curvatures);

  void braking_limiter(std::vector<PathPoint> &points, std::vector<double> &velocities,
                       const std::vector<double> &curvatures);
};