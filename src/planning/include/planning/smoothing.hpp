#ifndef SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_

#include <osqp.h>

#include <Eigen/Dense>
#include <cmath>
#include <functional>
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

  /**
   * @brief Adds curvature penalty terms to the quadratic objective function.
   *
   * @param num_path_points Number of points in the path
   * @param circular_index Lambda function for circular array indexing
   * @param add_coefficient Lambda function to add coefficients to the objective matrix
   */
  void add_curvature_terms(int num_path_points, const std::function<int(int)>& circular_index,
                           const std::function<void(int, int, double)>& add_coefficient) const;

  /**
   * @brief Adds smoothness penalty terms to the quadratic objective function.
   *
   * @param num_path_points Number of points in the path
   * @param circular_index Lambda function for circular array indexing
   * @param add_coefficient Lambda function to add coefficients to the objective matrix
   */
  void add_smoothness_terms(int num_path_points, const std::function<int(int)>& circular_index,
                            const std::function<void(int, int, double)>& add_coefficient) const;

  /**
   * @brief Adds penalty terms for slack variables to the quadratic objective function.
   *
   * @param num_path_points Number of points in the path
   * @param add_coefficient Lambda function to add coefficients to the objective matrix
   */
  void add_slack_penalty_terms(int num_path_points,
                               const std::function<void(int, int, double)>& add_coefficient) const;

  /**
   * @brief Adds penalty terms for slack variables to the quadratic objective function.
   *
   * @param quadratic_terms Map storing quadratic coefficient terms
   * @param num_path_points Number of points in the path
   * @param add_coefficient Lambda function to add coefficients to the objective matrix
   */
  void add_slack_penalty_terms(std::map<std::pair<int, int>, double>& quadratic_terms,
                               int num_path_points,
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
   * @param num_path_points Number of points in the path
   * @param safety_margin Safety distance from track boundaries
   */
  void add_boundary_constraints(std::vector<OSQPFloat>& constraint_values,
                                std::vector<OSQPInt>& constraint_row_indices,
                                std::vector<OSQPInt>& constraint_col_indices,
                                std::vector<OSQPFloat>& constraint_lower_bounds,
                                std::vector<OSQPFloat>& constraint_upper_bounds,
                                int& constraint_count, const std::vector<PathPoint>& left,
                                const std::vector<PathPoint>& right, int num_path_points,
                                double safety_margin) const;

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
   * @brief Converts sparse matrix data from coordinate format to Compressed Sparse Column (CSC)
   * format. CSC format is required by the OSQP solver for efficient matrix operations.
   *
   * @param values Non-zero values in coordinate format
   * @param row_indices Row indices in coordinate format
   * @param col_indices Column indices in coordinate format
   * @param total_variables Number of columns in the matrix
   * @param csc_x Output: non-zero values in CSC format
   * @param csc_i Output: row indices in CSC format
   * @param csc_p Output: column pointers in CSC format
   */
  void convert_to_csc_format(const std::vector<OSQPFloat>& values,
                             const std::vector<OSQPInt>& row_indices,
                             const std::vector<OSQPInt>& col_indices, int total_variables,
                             std::vector<OSQPFloat>& csc_x, std::vector<OSQPInt>& csc_i,
                             std::vector<OSQPInt>& csc_p) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_SMOOTHING2_HPP_