#include "planning/smoothing.hpp"

std::vector<PathPoint> PathSmoothing::smooth_path(std::vector<PathPoint>& path,
                                                  bool is_path_closed) const {
  if (!config_.use_path_smoothing_) {
    return path;
  }
  if (is_path_closed) {
    path.push_back(path[0]);
  }
  std::vector<PathPoint> result_path = filter_path(::fit_spline(
      path, config_.spline_precision_, config_.spline_order_, config_.spline_coeffs_ratio_));

  return result_path;
}

std::vector<PathPoint> PathSmoothing::optimize_path(std::vector<PathPoint>& path,
                                                    std::vector<PathPoint>& yellow_cones,
                                                    std::vector<PathPoint>& blue_cones,
                                                    bool is_path_closed) {

  if (!config_.use_optimization_) {
    return smooth_path(path, is_path_closed);
  }

  auto splines = ::fit_triple_spline(path, blue_cones, yellow_cones, config_.spline_precision_,
                                     config_.spline_order_);
  const std::vector<PathPoint> optimize_path =
      osqp_optimization(splines.center, splines.left, splines.right, is_path_closed);
  std::vector<PathPoint> filtered_path = filter_path(optimize_path);

  return splines.center;
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
    const std::function<void(int, int, double)>& add_coefficient, bool is_path_closed) const {
  int start = 1;
  int end = num_path_points - 1;

  if (is_path_closed) {
    start = 0;
    end = num_path_points;
  }

  // -------- ADD CURVATURE PENALTY TERMS --------
  // Penalize second-order differences to minimize curvature.
  // For each point, we penalize: (p[i-1] - 2*p[i] + p[i+1])^2
  //
  // OSQP minimizes (1/2) x^T P x, so all P coefficients must be multiplied by 2
  // to get the true mathematical penalty weight.
  //
  // For off-diagonal entries (i != j), OSQP only reads the upper triangle and
  // mirrors it internally — it does NOT double off-diagonal entries. So we pass
  // the full mathematical coefficient directly for off-diagonal terms.
  //
  // Expanding (a - 2b + c)^2:
  //   a^2  -> coeff +1w  -> pass +2w  (diagonal, divide by OSQP's 1/2 factor)
  //   b^2  -> coeff +4w  -> pass +8w
  //   c^2  -> coeff +1w  -> pass +2w
  //   ab   -> coeff -4w  -> pass -4w  (off-diagonal, OSQP mirrors so no extra factor)
  //   bc   -> coeff -4w  -> pass -4w
  //   ac   -> coeff +2w  -> pass +2w

  for (int point_idx = start; point_idx < end; ++point_idx) {
    int prev_point = circular_index(point_idx - 1);
    int next_point = circular_index(point_idx + 1);

    // X-coordinate curvature terms
    int x_prev    = 2 * prev_point;
    int x_current = 2 * point_idx;
    int x_next    = 2 * next_point;

    // FIX: diagonal terms ×2, off-diagonal terms use full mathematical coefficient
    add_coefficient(x_prev,    x_prev,    2 * config_.curvature_weight_);
    add_coefficient(x_current, x_current, 8 * config_.curvature_weight_);
    add_coefficient(x_next,    x_next,    2 * config_.curvature_weight_);
    add_coefficient(x_prev,    x_current, -4 * config_.curvature_weight_);
    add_coefficient(x_current, x_next,    -4 * config_.curvature_weight_);
    add_coefficient(x_prev,    x_next,     2 * config_.curvature_weight_);

    // Y-coordinate curvature terms
    int y_prev    = 2 * prev_point    + 1;
    int y_current = 2 * point_idx     + 1;
    int y_next    = 2 * next_point    + 1;

    add_coefficient(y_prev,    y_prev,    2 * config_.curvature_weight_);
    add_coefficient(y_current, y_current, 8 * config_.curvature_weight_);
    add_coefficient(y_next,    y_next,    2 * config_.curvature_weight_);
    add_coefficient(y_prev,    y_current, -4 * config_.curvature_weight_);
    add_coefficient(y_current, y_next,    -4 * config_.curvature_weight_);
    add_coefficient(y_prev,    y_next,     2 * config_.curvature_weight_);
  }
}

void PathSmoothing::add_slack_penalty_terms(
    int num_path_points, const std::function<void(int, int, double)>& add_coefficient) const {
  // -------- ADD SLACK VARIABLE PENALTY TERMS --------
  // FIX: multiply by 2 to account for OSQP's (1/2) x^T P x objective scaling
  const int num_slack_variables = num_path_points;
  for (int slack_idx = 0; slack_idx < num_slack_variables; ++slack_idx) {
    int slack_variable_index = 2 * num_path_points + slack_idx;
    add_coefficient(slack_variable_index, slack_variable_index, 2 * config_.safety_weight_);
  }
}

void PathSmoothing::add_proximity_terms(
    int num_path_points, const std::vector<PathPoint>& center,
    const std::function<void(int, int, double)>& add_quadratic_coefficient,
    std::vector<OSQPFloat>& linear_objective) const {
  // -------- ADD PROXIMITY TERMS --------
  // Penalize deviation from the center path to prevent degenerate solutions.
  // Without this, the solver can collapse all points toward the origin (zero
  // curvature = globally optimal but useless).
  //
  // Adds: proximity_weight * (x - cx)^2 = proximity_weight * x^2
  //                                       - 2 * proximity_weight * cx * x + const
  //
  // Quadratic part: FIX diagonal by ×2 for OSQP's (1/2) factor
  // Linear part: q[i] = -2 * proximity_weight * center[i]  (no factor needed here)

  for (int i = 0; i < num_path_points; ++i) {
    add_quadratic_coefficient(2 * i,     2 * i,     2 * 0.001* config_.curvature_weight_);
    add_quadratic_coefficient(2 * i + 1, 2 * i + 1, 2 * 0.001 * config_.curvature_weight_);

    linear_objective[2 * i] = -2.0 * 0.001 * config_.curvature_weight_ * center[i].position.x;
    linear_objective[2 * i + 1] = -2.0 * 0.001 * config_.curvature_weight_ * center[i].position.y;
  }
}

void PathSmoothing::add_boundary_constraints(
    std::vector<OSQPFloat>& constraint_values, std::vector<OSQPInt>& constraint_row_indices,
    std::vector<OSQPInt>& constraint_col_indices, std::vector<OSQPFloat>& constraint_lower_bounds,
    std::vector<OSQPFloat>& constraint_upper_bounds, int& constraint_count,
    const std::vector<PathPoint>& left, const std::vector<PathPoint>& right, int num_path_points,
    double safety_margin) const {
  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    Eigen::Vector2d left_boundary_point(left[point_idx].position.x, left[point_idx].position.y);
    Eigen::Vector2d right_boundary_point(right[point_idx].position.x, right[point_idx].position.y);

    Eigen::Vector2d lateral_direction = (left_boundary_point - right_boundary_point).normalized();

    if (lateral_direction.norm() < 1e-6) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Degenerate boundary at point %d", point_idx);
      continue;
    }

    double right_bound = right_boundary_point.dot(lateral_direction) + safety_margin;
    double left_bound  = left_boundary_point.dot(lateral_direction) - safety_margin;

    // FIX: Instead of skipping narrow corridors (which leaves gaps in the
    // constraint matrix and causes OSQP setup status 1), clamp to a minimal
    // feasible corridor centered at the midpoint.
    if (right_bound >= left_bound) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Point %d: corridor too narrow (right=%.3f >= left=%.3f), clamping to midpoint",
                  point_idx, right_bound, left_bound);
      double mid = (right_bound + left_bound) / 2.0;
      right_bound = mid - 0.01;
      left_bound  = mid + 0.01;
      // Do NOT skip — always add the constraint to keep matrix dimensions consistent
    }

    int slack_idx = 2 * num_path_points + point_idx;

    // RIGHT: n·p + s >= right_bound
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx);
    constraint_values.push_back(lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx + 1);
    constraint_values.push_back(lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(slack_idx);
    constraint_values.push_back(1.0);

    constraint_lower_bounds.push_back(right_bound);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;

    // LEFT: n·p - s <= left_bound
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx);
    constraint_values.push_back(lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx + 1);
    constraint_values.push_back(lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(slack_idx);
    constraint_values.push_back(-1.0);

    constraint_lower_bounds.push_back(-OSQP_INFTY);
    constraint_upper_bounds.push_back(left_bound);
    constraint_count++;
  }
}

void PathSmoothing::add_slack_nonnegativity_constraints(
    std::vector<OSQPFloat>& constraint_values, std::vector<OSQPInt>& constraint_row_indices,
    std::vector<OSQPInt>& constraint_col_indices, std::vector<OSQPFloat>& constraint_lower_bounds,
    std::vector<OSQPFloat>& constraint_upper_bounds, int& constraint_count,
    int num_path_points) const {
  // -------- ADD SLACK VARIABLE NON-NEGATIVITY CONSTRAINTS --------
  const int num_slack_variables = num_path_points;
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
  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> columns(total_variables);
  for (size_t entry = 0; entry < values.size(); ++entry) {
    columns[col_indices[entry]].push_back({row_indices[entry], values[entry]});
  }

  for (int col = 0; col < total_variables; ++col) {
    std::sort(columns[col].begin(), columns[col].end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
  }

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
                                                        const std::vector<PathPoint>& right,
                                                        bool is_path_closed) const {
  if (center.size() != left.size() || center.size() != right.size() ||
      left.size() != center.size()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "The splines have different sizes. Right - %ld, Left - %ld, Center - %ld",
                center.size(), left.size(), right.size());
  }
  const int num_path_points = center.size();

  if (num_path_points < 5) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Too few points for OSQP optimization (%d points). Minimum is 5.", num_path_points);
    return center;
  }

  const double safety_margin = config_.car_width_ / 2 + config_.safety_margin_;

  auto circular_index = [&](int i) -> int {
    if (is_path_closed) {
      return (i + num_path_points) % num_path_points;
    }
    return std::clamp(i, 0, num_path_points - 1);
  };

  const int num_slack_variables = num_path_points;
  const int total_variables = 2 * num_path_points + num_slack_variables;

  std::map<std::pair<int, int>, double> quadratic_terms;

  auto add_quadratic_coefficient = [&](int row_idx, int col_idx, double coefficient) {
    if (row_idx > col_idx) {
      std::swap(row_idx, col_idx);
    }
    quadratic_terms[{row_idx, col_idx}] += coefficient;
  };

  // -------- BUILD LINEAR OBJECTIVE VECTOR (q) --------
  // Allocated before curvature/proximity terms so proximity can write into it
  std::vector<OSQPFloat> linear_objective(total_variables, 0.0);

  add_curvature_terms(num_path_points, circular_index, add_quadratic_coefficient, is_path_closed);
  add_slack_penalty_terms(num_path_points, add_quadratic_coefficient);

  // FIX: add proximity terms to anchor points near center path and prevent
  // the degenerate zero-curvature solution
  add_proximity_terms(num_path_points, center, add_quadratic_coefficient, linear_objective);

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

  std::vector<OSQPFloat> P_x;
  std::vector<OSQPInt>   P_i, P_p;
  convert_to_csc_format(P_values, P_row_indices, P_col_indices, total_variables, P_x, P_i, P_p);

  std::vector<OSQPFloat> A_x;
  std::vector<OSQPInt>   A_i, A_p;
  convert_to_csc_format(constraint_values, constraint_row_indices, constraint_col_indices,
                        total_variables, A_x, A_i, A_p);

  OSQPCscMatrix objective_matrix;
  objective_matrix.m    = total_variables;
  objective_matrix.n    = total_variables;
  objective_matrix.nzmax = P_x.size();
  objective_matrix.nz   = -1;
  objective_matrix.x    = P_x.data();
  objective_matrix.i    = P_i.data();
  objective_matrix.p    = P_p.data();

  OSQPCscMatrix constraint_matrix;
  constraint_matrix.m    = total_constraints;
  constraint_matrix.n    = total_variables;
  constraint_matrix.nzmax = A_x.size();
  constraint_matrix.nz   = -1;
  constraint_matrix.x    = A_x.data();
  constraint_matrix.i    = A_i.data();
  constraint_matrix.p    = A_p.data();

  OSQPSettings solver_settings;
  ::osqp_set_default_settings(&solver_settings);
  solver_settings.verbose  = false;
  solver_settings.max_iter = config_.max_iterations_;
  solver_settings.eps_abs  = config_.tolerance_;
  solver_settings.eps_rel  = config_.tolerance_;
  solver_settings.polishing = 1;

  OSQPInt solve_status = 0;

  if (solver_ != nullptr) {
    ::osqp_cleanup(solver_);
    solver_ = nullptr;
  }

  OSQPInt setup_status =
      ::osqp_setup(&solver_, &objective_matrix, linear_objective.data(), &constraint_matrix,
                   constraint_lower_bounds.data(), constraint_upper_bounds.data(), total_variables,
                   total_constraints, &solver_settings);

  if (setup_status != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OSQP setup failed with status %lld", setup_status);
    ::osqp_cleanup(solver_);
    solver_ = nullptr;
    return center;
  }

  // -------- WARM START --------
  if (!cached_primal_.empty()) {
    std::vector<OSQPFloat> warm_x(total_variables, 0.0);
    std::vector<OSQPFloat> warm_y(total_constraints, 0.0);

    const int old_num    = cached_num_points_;
    const int reuse_num  = std::min(old_num, num_path_points);

    // Primal: reuse old coords, seed new points from center
    for (int i = 0; i < reuse_num; ++i) {
      warm_x[2 * i]     = cached_primal_[2 * i];
      warm_x[2 * i + 1] = cached_primal_[2 * i + 1];
    }
    for (int i = reuse_num; i < num_path_points; ++i) {
      warm_x[2 * i]     = static_cast<OSQPFloat>(center[i].position.x);
      warm_x[2 * i + 1] = static_cast<OSQPFloat>(center[i].position.y);
    }

    // FIX: also reuse dual variables (boundary and slack duals)
    // Constraint layout: [right_0, left_0, ..., right_n, left_n, slack_0, ..., slack_n]
    if (!cached_dual_.empty()) {
      const int old_slack_offset = 2 * old_num;
      const int new_slack_offset = 2 * num_path_points;

      for (int i = 0; i < reuse_num; ++i) {
        warm_y[2 * i]     = cached_dual_[2 * i];      // right boundary dual
        warm_y[2 * i + 1] = cached_dual_[2 * i + 1];  // left boundary dual
      }
      for (int i = 0; i < reuse_num; ++i) {
        warm_y[new_slack_offset + i] = cached_dual_[old_slack_offset + i];  // slack dual
      }
      // New points left at zero — no prior info available
    }

    osqp_warm_start(solver_, warm_x.data(), warm_y.data());
  }

  solve_status = ::osqp_solve(solver_);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OSQP status: %s | iterations: %lld | obj: %.4f",
              solver_->info->status, solver_->info->iter, solver_->info->obj_val);

  // FIX: The old threshold of 1.0 was completely wrong for this problem scale.
  // With proximity terms now present, a valid solution will have a meaningful
  // positive objective. A degenerate solution (all points collapsed) will still
  // produce near-zero even with proximity terms only if proximity_weight_ is
  // very small. Use a small epsilon check instead and rely on solver status.
  if (solver_->info->status_val != 1 /* OSQP_SOLVED */ &&
      solver_->info->status_val != 2 /* OSQP_SOLVED_INACCURATE */) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "OSQP did not converge (status: %s), returning original path",
                solver_->info->status);
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    return center;
  }

  std::vector<PathPoint> optimized_path = center;

  if ((solver_->solution == nullptr) || (solver_->solution->x == nullptr)) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "OSQP solution is null, returning original path");
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    return optimized_path;
  }

  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    optimized_path[point_idx].position.x = solver_->solution->x[2 * point_idx];
    optimized_path[point_idx].position.y = solver_->solution->x[2 * point_idx + 1];
  }

  // Cache solution for next call
  cached_num_points_ = num_path_points;
  cached_is_closed_  = is_path_closed;
  cached_primal_.assign(solver_->solution->x, solver_->solution->x + total_variables);
  cached_dual_.assign(solver_->solution->y,   solver_->solution->y + total_constraints);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "OSQP optimization completed successfully with %d points (status: %lld)",
               num_path_points, solve_status);

  return optimized_path;
}