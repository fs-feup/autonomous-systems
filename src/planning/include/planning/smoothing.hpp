#ifndef SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <map>
#include <vector>

#include "common_lib/structures/path_point.hpp"
#include "config/smoothing_config.hpp"
#include "osqp.h"
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
   * @brief Smooths a path by fitting a B-spline through the input points.
   *
   * @param path The input path to be smoothed
   * @param is_path_closed Whether the path is closed
   * @return std::vector<PathPoint> The smoothed path, with closure point appended if applicable
   */
  std::vector<PathPoint> smooth_path(std::vector<PathPoint>& path, bool is_path_closed) const;

  /**
   * @brief Optimizes a racing line path using the minimum curvature
   *
   * @param path The initial center path to be optimized
   * @param yellow_cones Track boundary markers on the right boundary
   * @param blue_cones Track boundary markers on the left boundary
   * @param is_path_closed Whether the path forms a closed loop
   * @param is_path_final If true, optimizes the full path as a closed loop (final lap)
   * @return std::vector<PathPoint> The optimized path
   */
  std::vector<PathPoint> optimize_path(std::vector<PathPoint>& path,
                                       const std::vector<PathPoint>& yellow_cones,
                                       const std::vector<PathPoint>& blue_cones,
                                       bool is_path_closed, bool is_path_final);

  /**
   * @brief Destroys the PathSmoothing object and cleans up the OSQP solver if allocated.
   */
  ~PathSmoothing() {
    if (solver_) {
      const OSQPInt status = ::osqp_cleanup(solver_);
      (void)status;
      solver_ = nullptr;
    }
  }

private:
  /**
   * @brief configuration of the smoothing algorithm
   *
   */
  PathSmoothingConfig config_;
  // Warm start cache
  mutable OSQPSolver* solver_ = nullptr;
  mutable int cached_num_points_ = -1;
  mutable bool cached_is_closed_ = false;
  mutable std::vector<OSQPFloat> cached_primal_;
  mutable std::vector<OSQPFloat> cached_dual_;

  /**
   * @brief Filters path points using a minimum spacing constraint.
   *
   * @param path Input path.
   * @return Filtered path.
   */
  std::vector<PathPoint> filter_path(const std::vector<PathPoint>& path) const;

  /**
   * @brief Optimizes a path using quadratic programming (OSQP).
   *
   * @param center Sequence of points representing the initial center line path
   * @param left Sequence of points representing the left track boundary
   * @param right Sequence of points representing the right track boundary
   * @param is_path_closed Whether the path forms a closed loop
   * @param is_final If true, optimizes the full path; otherwise uses a sliding window
   * @return std::vector<PathPoint> Optimized path
   */
  std::vector<PathPoint> osqp_optimization(const std::vector<PathPoint>& center,
                                           const std::vector<PathPoint>& left,
                                           const std::vector<PathPoint>& right, bool is_path_closed,
                                           bool is_final) const;

  /**
   * @brief Adds curvature penalty terms to the quadratic objective function.
   *
   * @param num_path_points Number of points in the path
   * @param circular_index Lambda function for circular array indexing
   * @param add_coefficient Lambda function to add coefficients to the objective matrix
   * @param is_path_closed Whether curvature should wrap at the endpoints
   */
  void add_curvature_terms(int num_path_points, const std::function<int(int)>& circular_index,
                           const std::function<void(int, int, double)>& add_coefficient,
                           bool is_path_closed) const;

  /**
   * @brief Adds penalty terms for slack variables to the quadratic objective function.
   *
   * @param num_path_points Number of points in the path
   * @param add_coefficient Lambda function to add coefficients to the objective matrix
   */
  void add_slack_penalty_terms(int num_path_points,
                               const std::function<void(int, int, double)>& add_coefficient) const;

  /**
   * @brief Adds track boundary constraints to ensure the optimized path stays within the track.
   *
   * @param constraint_values Non-zero values in the constraint matrix
   * @param constraint_row_indices Row indices for constraint matrix entries
   * @param constraint_col_indices Column indices for constraint matrix entries
   * @param constraint_lower_bounds Lower bounds for each constraint
   * @param constraint_upper_bounds Upper bounds for each constraint
   * @param constraint_count Running count of constraints added
   * @param left Left track boundary points
   * @param right Right track boundary points
   * @param center Center line points, used to derive the forward direction
   * @param num_path_points Number of points in the path
   * @param safety_margin Safety distance from track boundaries
   */
  void add_boundary_constraints(
      std::vector<OSQPFloat>& constraint_values, std::vector<OSQPInt>& constraint_row_indices,
      std::vector<OSQPInt>& constraint_col_indices, std::vector<OSQPFloat>& constraint_lower_bounds,
      std::vector<OSQPFloat>& constraint_upper_bounds, int& constraint_count,
      const std::vector<PathPoint>& left, const std::vector<PathPoint>& right,
      const std::vector<PathPoint>& center, int num_path_points, double safety_margin) const;

  /**
   * @brief Adds non-negativity constraints for slack variables.
   *
   * @param constraint_values Non-zero values in the constraint matrix
   * @param constraint_row_indices Row indices for constraint matrix entries
   * @param constraint_col_indices Column indices for constraint matrix entries
   * @param constraint_lower_bounds Lower bounds for each constraint
   * @param constraint_upper_bounds Upper bounds for each constraint
   * @param constraint_count Running count of constraints added
   * @param num_path_points Number of points in the path
   */
  void add_slack_nonnegativity_constraints(std::vector<OSQPFloat>& constraint_values,
                                           std::vector<OSQPInt>& constraint_row_indices,
                                           std::vector<OSQPInt>& constraint_col_indices,
                                           std::vector<OSQPFloat>& constraint_lower_bounds,
                                           std::vector<OSQPFloat>& constraint_upper_bounds,
                                           int& constraint_count, int num_path_points) const;

  /**
   * @brief Adds proximity penalty terms to keep the optimized path near the center line.
   *
   * @param num_path_points Number of points in the path
   * @param center Center line reference points
   * @param add_quadratic_coefficient Lambda to add quadratic objective coefficients
   * @param linear_objective Linear objective vector (modified in-place)
   */
  void add_proximity_terms(int num_path_points, const std::vector<PathPoint>& center,
                           const std::function<void(int, int, double)>& add_quadratic_coefficient,
                           std::vector<OSQPFloat>& linear_objective) const;

  /**
   * @brief Converts sparse matrix data from coordinate format to Compressed Sparse Column (CSC)
   * format required by the OSQP solver.
   *
   * @param values Non-zero values in coordinate format
   * @param row_indices Row indices in coordinate format
   * @param col_indices Column indices in coordinate format
   * @param total_variables Number of columns in the matrix
   * @param csc_values Output: non-zero values in CSC format
   * @param csc_row_indices Output: row indices in CSC format
   * @param csc_col_pointers Output: column pointers in CSC format
   */
  void convert_to_csc_format(const std::vector<OSQPFloat>& values,
                             const std::vector<OSQPInt>& row_indices,
                             const std::vector<OSQPInt>& col_indices, int total_variables,
                             std::vector<OSQPFloat>& csc_values,
                             std::vector<OSQPInt>& csc_row_indices,
                             std::vector<OSQPInt>& csc_col_pointers) const;

  /**
   * @brief Core OSQP optimization implementation operating on a given window.
   *
   * @param center Sequence of points representing the initial center line path
   * @param left Sequence of points representing the left track boundary
   * @param right Sequence of points representing the right track boundary
   * @param opt_start Index into center/left/right where the optimization window starts
   * @param is_path_closed Whether the path forms a closed loop
   * @return std::vector<PathPoint> Optimized path
   */
  std::vector<PathPoint> osqp_optimization_implementation(
      const std::vector<PathPoint>& center_path, const std::vector<PathPoint>& left_boundary,
      const std::vector<PathPoint>& right_boundary, bool is_path_closed) const;
  // TODO: dioxigen
  void build_warm_start(int total_variables, int num_path_points, int total_constraints,
                        const std::vector<PathPoint>& center_path) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING_HPP_