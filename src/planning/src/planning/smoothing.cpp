#include "planning/smoothing.hpp"

std::vector<PathPoint> PathSmoothing::smooth_path(const std::vector<PathPoint>& path) const {
  if (!config_.use_path_smoothing_) {
    return path;
  }
  std::vector<PathPoint> result_path = ::fit_spline(
      path, config_.spline_precision_, config_.spline_order_, config_.spline_coeffs_ratio_);
  return filter_path(result_path);
}

std::vector<PathPoint> PathSmoothing::optimize_path(
    const std::vector<PathPoint>& path, const std::vector<PathPoint>& yellow_cones,
    const std::vector<PathPoint>& blue_cones) const {
  if (!config_.use_optimization_) {
    std::vector<PathPoint> result = smooth_path(path);
    // Close the loop by adding the first point again
    result.push_back(result.front());
    return result;
  }

  auto splines = ::fit_triple_spline(path, blue_cones, yellow_cones, config_.spline_precision_,
                                     config_.spline_order_);

  const std::vector<PathPoint> optimize_path =
      osqp_optimization(splines.center, splines.left, splines.right);
  std::vector<PathPoint> filtered_path = filter_path(optimize_path);

  // Close the loop by adding the first point again
  filtered_path.push_back(filtered_path.front());

  return filtered_path;
}

std::vector<PathPoint> PathSmoothing::filter_path(const std::vector<PathPoint>& path) const {
  std::vector<PathPoint> filtered;
  for (const auto& p : path) {
    if (filtered.empty() || filtered.back().position.euclidean_distance(p.position) >
                                config_.min_path_point_distance_) {
      filtered.push_back(p);
    }
  }
  return filtered;
}

void PathSmoothing::add_curvature_terms(
    int num_path_points, const std::function<int(int)>& circular_index,
    const std::function<void(int, int, double)>& add_coefficient) const {
  // -------- ADD CURVATURE PENALTY TERMS --------
  // Penalize second-order differences to minimize curvature
  // For each point, we penalize: (p[i-1] - 2*p[i] + p[i+1])^2
  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    int prev_point = circular_index(point_idx - 1);
    int next_point = circular_index(point_idx + 1);

    // X-coordinate curvature terms
    int x_prev = 2 * prev_point;
    int x_current = 2 * point_idx;
    int x_next = 2 * next_point;

    add_coefficient(x_prev, x_prev, config_.curvature_weight_);
    add_coefficient(x_current, x_current, 4 * config_.curvature_weight_);
    add_coefficient(x_next, x_next, config_.curvature_weight_);
    add_coefficient(x_prev, x_current, -2 * config_.curvature_weight_);
    add_coefficient(x_current, x_next, -2 * config_.curvature_weight_);
    add_coefficient(x_prev, x_next, config_.curvature_weight_);

    // Y-coordinate curvature terms
    int y_prev = 2 * prev_point + 1;
    int y_current = 2 * point_idx + 1;
    int y_next = 2 * next_point + 1;

    add_coefficient(y_prev, y_prev, config_.curvature_weight_);
    add_coefficient(y_current, y_current, 4 * config_.curvature_weight_);
    add_coefficient(y_next, y_next, config_.curvature_weight_);
    add_coefficient(y_prev, y_current, -2 * config_.curvature_weight_);
    add_coefficient(y_current, y_next, -2 * config_.curvature_weight_);
    add_coefficient(y_prev, y_next, config_.curvature_weight_);
  }
}

void PathSmoothing::add_smoothness_terms(
    int num_path_points, const std::function<int(int)>& circular_index,
    const std::function<void(int, int, double)>& add_coefficient) const {
  // -------- ADD SMOOTHNESS PENALTY TERMS --------
  // Penalize first-order differences to minimize jerk
  // For each consecutive pair, we penalize: (p[i+1] - p[i])^2
  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    int next_point = circular_index(point_idx + 1);

    // X-coordinate smoothness terms
    int x_current = 2 * point_idx;
    int x_next = 2 * next_point;
    add_coefficient(x_current, x_current, config_.smoothness_weight_);
    add_coefficient(x_next, x_next, config_.smoothness_weight_);
    add_coefficient(x_current, x_next, -config_.smoothness_weight_);

    // Y-coordinate smoothness terms
    int y_current = 2 * point_idx + 1;
    int y_next = 2 * next_point + 1;
    add_coefficient(y_current, y_current, config_.smoothness_weight_);
    add_coefficient(y_next, y_next, config_.smoothness_weight_);
    add_coefficient(y_current, y_next, -config_.smoothness_weight_);
  }
}

void PathSmoothing::add_slack_penalty_terms(
    int num_path_points, const std::function<void(int, int, double)>& add_coefficient) const {
  // -------- ADD SLACK VARIABLE PENALTY TERMS --------
  // Penalize slack variables to encourage staying within bounds
  const int num_slack_variables = 2 * num_path_points;
  for (int slack_idx = 0; slack_idx < num_slack_variables; ++slack_idx) {
    int slack_variable_index = 2 * num_path_points + slack_idx;
    add_coefficient(slack_variable_index, slack_variable_index, config_.safety_weight_);
  }
}

void PathSmoothing::add_boundary_constraints(
    std::vector<OSQPFloat>& constraint_values, std::vector<OSQPInt>& constraint_row_indices,
    std::vector<OSQPInt>& constraint_col_indices, std::vector<OSQPFloat>& constraint_lower_bounds,
    std::vector<OSQPFloat>& constraint_upper_bounds, int& constraint_count,
    const std::vector<PathPoint>& left, const std::vector<PathPoint>& right, int num_path_points,
    double safety_margin) const {
  // -------- ADD TRACK BOUNDARY CONSTRAINTS --------
  // For each point, ensure it stays within the left and right boundaries
  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    Eigen::Vector2d left_boundary_point(left[point_idx].position.x, left[point_idx].position.y);
    Eigen::Vector2d right_boundary_point(right[point_idx].position.x, right[point_idx].position.y);

    // Compute lateral direction (perpendicular to track)
    Eigen::Vector2d lateral_direction = (left_boundary_point - right_boundary_point).normalized();

    // Right boundary constraint: lateral_direction · point + slack >= right_boundary_value
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx);
    constraint_values.push_back(lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx + 1);
    constraint_values.push_back(lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * num_path_points + 2 * point_idx);
    constraint_values.push_back(1.0);

    double right_boundary_constraint = right_boundary_point.dot(lateral_direction) + safety_margin;
    constraint_lower_bounds.push_back(right_boundary_constraint);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;

    // Left boundary constraint: -lateral_direction · point + slack >= -left_boundary_value
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx);
    constraint_values.push_back(-lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx + 1);
    constraint_values.push_back(-lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * num_path_points + 2 * point_idx + 1);
    constraint_values.push_back(1.0);

    double left_boundary_constraint = -left_boundary_point.dot(lateral_direction) + safety_margin;
    constraint_lower_bounds.push_back(left_boundary_constraint);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;
  }
}

void PathSmoothing::add_slack_nonnegativity_constraints(
    std::vector<OSQPFloat>& constraint_values, std::vector<OSQPInt>& constraint_row_indices,
    std::vector<OSQPInt>& constraint_col_indices, std::vector<OSQPFloat>& constraint_lower_bounds,
    std::vector<OSQPFloat>& constraint_upper_bounds, int& constraint_count,
    int num_path_points) const {
  // -------- ADD SLACK VARIABLE NON-NEGATIVITY CONSTRAINTS --------
  // Ensure all slack variables are non-negative
  const int num_slack_variables = 2 * num_path_points;
  for (int slack_idx = 0; slack_idx < num_slack_variables; ++slack_idx) {
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * num_path_points + slack_idx);
    constraint_values.push_back(1.0);

    constraint_lower_bounds.push_back(0.0);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;
  }
}

void PathSmoothing::convert_to_csc_format(const std::vector<OSQPFloat>& values,
                                          const std::vector<OSQPInt>& row_indices,
                                          const std::vector<OSQPInt>& col_indices,
                                          int total_variables, std::vector<OSQPFloat>& csc_x,
                                          std::vector<OSQPInt>& csc_i,
                                          std::vector<OSQPInt>& csc_p) const {
  // Convert coordinate format to CSC (Compressed Sparse Column) format
  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> columns(total_variables);
  for (size_t entry = 0; entry < values.size(); ++entry) {
    columns[col_indices[entry]].push_back({row_indices[entry], values[entry]});
  }

  // Sort each column by row index
  for (int col = 0; col < total_variables; ++col) {
    std::sort(columns[col].begin(), columns[col].end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
  }

  // Build CSC arrays
  csc_x.reserve(values.size());
  csc_i.reserve(values.size());
  csc_p.resize(total_variables + 1);

  csc_p[0] = 0;
  for (int col = 0; col < total_variables; ++col) {
    for (const auto& [row, value] : columns[col]) {
      csc_i.push_back(row);
      csc_x.push_back(value);
    }
    csc_p[col + 1] = csc_x.size();
  }
}

std::vector<PathPoint> PathSmoothing::osqp_optimization(const std::vector<PathPoint>& center,
                                                        const std::vector<PathPoint>& left,
                                                        const std::vector<PathPoint>& right) const {
  if (center.size() != left.size() || center.size() != right.size() ||
      left.size() != center.size()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "The splines have different sizes. Right - %d, Left - %d, Center - %d",
                center.size(), left.size(), right.size());
  }
  const int num_path_points = center.size();

  // Check if we have enough points for optimization
  if (num_path_points < 5) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Too few points for OSQP optimization (%d points). Minimum is 5.", num_path_points);
    return center;
  }

  // -------- COMPUTE SAFETY MARGIN --------
  const double safety_margin = config_.car_width_ / 2 + config_.safety_margin_;

  // Helper lambda for circular indexing (wraps around the path)
  auto circular_index = [&](int i) { return (i + num_path_points) % num_path_points; };

  // -------- DEFINE OPTIMIZATION VARIABLES --------
  // Decision variables: 2 coordinates (x,y) per point + 2 slack variables per point
  const int num_slack_variables = 2 * num_path_points;
  const int total_variables = 2 * num_path_points + num_slack_variables;

  // -------- BUILD QUADRATIC OBJECTIVE MATRIX (P) --------
  // Store quadratic terms in a map for easy accumulation, then convert to sparse format
  std::map<std::pair<int, int>, double> quadratic_terms;

  // Helper lambda to add coefficients to the upper triangular part of P
  auto add_quadratic_coefficient = [&](int row_idx, int col_idx, double coefficient) {
    if (row_idx > col_idx) {
      std::swap(row_idx, col_idx);
    }
    quadratic_terms[{row_idx, col_idx}] += coefficient;
  };

  add_curvature_terms(num_path_points, circular_index, add_quadratic_coefficient);
  add_smoothness_terms(num_path_points, circular_index, add_quadratic_coefficient);
  add_slack_penalty_terms(num_path_points, add_quadratic_coefficient);

  // -------- BUILD LINEAR OBJECTIVE VECTOR (q) --------
  // All zeros since we only have quadratic terms in the objective
  std::vector<OSQPFloat> linear_objective(total_variables, 0.0);

  // -------- CONVERT QUADRATIC TERMS TO SPARSE FORMAT --------
  std::vector<OSQPFloat> P_values;
  std::vector<OSQPInt> P_row_indices, P_col_indices;

  P_values.reserve(quadratic_terms.size());
  P_row_indices.reserve(quadratic_terms.size());
  P_col_indices.reserve(quadratic_terms.size());

  for (const auto& [indices, value] : quadratic_terms) {
    P_row_indices.push_back(indices.first);
    P_col_indices.push_back(indices.second);
    P_values.push_back(value);
  }

  // -------- BUILD CONSTRAINT MATRIX (A) AND BOUNDS --------
  std::vector<OSQPFloat> constraint_values;
  std::vector<OSQPInt> constraint_row_indices, constraint_col_indices;
  std::vector<OSQPFloat> constraint_lower_bounds, constraint_upper_bounds;

  int constraint_count = 0;

  add_boundary_constraints(constraint_values, constraint_row_indices, constraint_col_indices,
                           constraint_lower_bounds, constraint_upper_bounds, constraint_count, left,
                           right, num_path_points, safety_margin);

  add_slack_nonnegativity_constraints(constraint_values, constraint_row_indices,
                                      constraint_col_indices, constraint_lower_bounds,
                                      constraint_upper_bounds, constraint_count, num_path_points);

  const int total_constraints = constraint_count;

  // Build CSC arrays for P
  std::vector<OSQPFloat> P_x;
  std::vector<OSQPInt> P_i;
  std::vector<OSQPInt> P_p;
  convert_to_csc_format(P_values, P_row_indices, P_col_indices, total_variables, P_x, P_i, P_p);

  // Build CSC arrays for A
  std::vector<OSQPFloat> A_x;
  std::vector<OSQPInt> A_i;
  std::vector<OSQPInt> A_p;
  convert_to_csc_format(constraint_values, constraint_row_indices, constraint_col_indices,
                        total_variables, A_x, A_i, A_p);

  // -------- POPULATE MATRIX STRUCTURES --------
  OSQPCscMatrix objective_matrix;
  objective_matrix.m = total_variables;
  objective_matrix.n = total_variables;
  objective_matrix.nzmax = P_x.size();
  objective_matrix.nz = -1;
  objective_matrix.x = P_x.data();
  objective_matrix.i = P_i.data();
  objective_matrix.p = P_p.data();

  OSQPCscMatrix constraint_matrix;
  constraint_matrix.m = total_constraints;
  constraint_matrix.n = total_variables;
  constraint_matrix.nzmax = A_x.size();
  constraint_matrix.nz = -1;
  constraint_matrix.x = A_x.data();
  constraint_matrix.i = A_i.data();
  constraint_matrix.p = A_p.data();

  // -------- CONFIGURE OSQP SOLVER SETTINGS --------
  OSQPSettings solver_settings;
  ::osqp_set_default_settings(&solver_settings);
  solver_settings.verbose = 1;
  solver_settings.max_iter = config_.max_iterations_;
  solver_settings.eps_abs = config_.tolerance_;
  solver_settings.eps_rel = config_.tolerance_;
  solver_settings.polishing = 1;

  // -------- SETUP OSQP SOLVER --------
  OSQPSolver* solver = nullptr;

  OSQPInt setup_status =
      ::osqp_setup(&solver, &objective_matrix, linear_objective.data(), &constraint_matrix,
                   constraint_lower_bounds.data(), constraint_upper_bounds.data(), total_variables,
                   total_constraints, &solver_settings);

  if (setup_status != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OSQP setup failed with status %lld", setup_status);
    if (solver) {
      (void)::osqp_cleanup(solver);
    }
    return center;
  }

  // -------- SOLVE THE OPTIMIZATION PROBLEM --------
  OSQPInt solve_status = ::osqp_solve(solver);

  // -------- EXTRACT OPTIMIZED PATH FROM SOLUTION --------
  std::vector<PathPoint> optimized_path = center;

  if ((solver->solution != nullptr) && (solver->solution->x != nullptr)) {
    for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
      optimized_path[point_idx].position.x = solver->solution->x[2 * point_idx];
      optimized_path[point_idx].position.y = solver->solution->x[2 * point_idx + 1];
    }

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "OSQP optimization completed successfully with %d points (status: %lld)",
                 num_path_points, solve_status);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "OSQP solution is null, returning original path");
  }

  // -------- CLEANUP OSQP RESOURCES --------
  (void)::osqp_cleanup(solver);

  return optimized_path;
}