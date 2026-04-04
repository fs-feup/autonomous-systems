#pragma once

#include <cmath>

#include "common_lib/structures/path_point.hpp"
#include "config/velocity_config.hpp"

using PathPoint = common_lib::structures::PathPoint;

/**
 * @brief Computes velocity profiles for a planned path based on curvature and dynamics constraints.
 *
 * The VelocityPlanning class generates a velocity profile along a path by:
 * - Estimating curvature using the Menger curvature formula (circle fitting through three points),
 * - Deriving maximum allowable velocities from lateral acceleration limits,
 * - Applying a friction ellipse model to account for combined longitudinal and lateral tire forces,
 * - Propagating acceleration constraints forward and braking constraints backward along the path.
 *
 * The velocity planner respects the friction circle constraint: a_x² + a_y² ≤ a_max²,
 * ensuring that the vehicle stays within tire grip limits during combined cornering and
 * acceleration/braking.
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
   * @brief Assigns an velocity to each point of the path.
   *
   * @param final_path Vector of path points to update with planned velocities.
   */
  void set_velocity(std::vector<PathPoint> &final_path);

  /**
   * @brief Computes velocity for trackdrive scenarios.
   *
   * @param final_path Vector of path points to update with planned velocities.
   */
  void trackdrive_velocity(std::vector<PathPoint> &final_path);

  /**
   * @brief Applies a braking velocity profile starting after a given braking distance.
   *
   * @param final_path Vector of path points to update with planned velocities.
   * @param braking_distance Distance along the path before braking begins.
   */
  void stop(std::vector<PathPoint> &final_path, double braking_distance);

private:
  /**
   * @brief configuration of the velocity planning algorithm
   *
   */
  VelocityPlanningConfig config_;

  /**
   * @brief Numerical tolerance for floating-point comparisons
   */
  static constexpr double epsilon = 1e-9;

  /**
   * @brief Computes the curvature at a point using the Menger curvature formula.
   *
   * @param p1 Previous point
   * @param p2 Current point where curvature is computed
   * @param p3 Next point
   * @return double Curvature value. Returns 0.0 for straight sections
   *               or nearly collinear points.
   */
  double find_curvature(const PathPoint &p1, const PathPoint &p2, const PathPoint &p3);

  /**
   * @brief Computes the maximum velocity at each point based on curvature constraints.
   *
   * @param curvatures Vector of curvature values for each path point
   * @param velocities Output vector to store the computed maximum velocities
   */
  void point_speed(const std::vector<double> &curvatures, std::vector<double> &velocities);

  /**
   * @brief Limits velocities based on forward acceleration constraints and friction ellipse.
   *
   * @param points Vector of path points (used for distance calculations)
   * @param velocities Vector of velocities to be updated (input/output)
   * @param curvatures Vector of curvature values (used to compute lateral acceleration)
   */
  void acceleration_limiter(const std::vector<PathPoint> &points, std::vector<double> &velocities,
                            const std::vector<double> &curvatures);

  /**
   * @brief Limits velocities based on backward braking constraints and friction ellipse.
   *
   * @param points Vector of path points (used for distance calculations)
   * @param velocities Vector of velocities to be updated (input/output)
   * @param curvatures Vector of curvature values (used to compute lateral acceleration)
   */
  void braking_limiter(std::vector<PathPoint> &points, std::vector<double> &velocities,
                       const std::vector<double> &curvatures);
};