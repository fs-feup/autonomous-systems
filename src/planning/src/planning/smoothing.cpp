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

  // auto splines = ::fit_triple_spline(path, blue_cones, yellow_cones, config_.spline_precision_,
  //                                    config_.spline_order_);
  //TODA: CHANGE THIS!!!!!!!
  const std::vector<PathPoint> optimize_path =
      osqp_optimization(path, blue_cones, yellow_cones, is_path_closed, false);
  std::vector<PathPoint> filtered_path = filter_path(optimize_path);

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
    int x_prev = 2 * prev_point;
    int x_current = 2 * point_idx;
    int x_next = 2 * next_point;

    add_coefficient(x_prev, x_prev, 2 * config_.curvature_weight_);
    add_coefficient(x_current, x_current, 8 * config_.curvature_weight_);
    add_coefficient(x_next, x_next, 2 * config_.curvature_weight_);
    add_coefficient(x_prev, x_current, -4 * config_.curvature_weight_);
    add_coefficient(x_current, x_next, -4 * config_.curvature_weight_);
    add_coefficient(x_prev, x_next, 2 * config_.curvature_weight_);

    // Y-coordinate curvature terms
    int y_prev = 2 * prev_point + 1;
    int y_current = 2 * point_idx + 1;
    int y_next = 2 * next_point + 1;

    add_coefficient(y_prev, y_prev, 2 * config_.curvature_weight_);
    add_coefficient(y_current, y_current, 8 * config_.curvature_weight_);
    add_coefficient(y_next, y_next, 2 * config_.curvature_weight_);
    add_coefficient(y_prev, y_current, -4 * config_.curvature_weight_);
    add_coefficient(y_current, y_next, -4 * config_.curvature_weight_);
    add_coefficient(y_prev, y_next, 2 * config_.curvature_weight_);
  }
}

void PathSmoothing::add_slack_penalty_terms(
    int num_path_points, const std::function<void(int, int, double)>& add_coefficient) const {
  // -------- ADD SLACK VARIABLE PENALTY TERMS --------
  // Multiply by 2 to account for OSQP's (1/2) x^T P x objective scaling
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
    add_quadratic_coefficient(2 * i, 2 * i, 2 * 0.001 * config_.curvature_weight_);
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
    double left_bound = left_boundary_point.dot(lateral_direction) - safety_margin;

    if (right_bound >= left_bound) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Point %d: corridor too narrow (right=%.3f >= left=%.3f), clamping to midpoint",
                  point_idx, right_bound, left_bound);
      double mid = (right_bound + left_bound) / 2.0;
      right_bound = mid - 0.01;
      left_bound = mid + 0.01;
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

// -------- ADD SEAM EQUALITY CONSTRAINTS --------
// Pins window point 0 exactly to center[opt_start], preventing the open seam
// from drifting away from the already-fixed prefix of the path.
void PathSmoothing::add_seam_constraints(std::vector<OSQPFloat>& constraint_values,
                                         std::vector<OSQPInt>& constraint_row_indices,
                                         std::vector<OSQPInt>& constraint_col_indices,
                                         std::vector<OSQPFloat>& constraint_lower_bounds,
                                         std::vector<OSQPFloat>& constraint_upper_bounds,
                                         int& constraint_count, const PathPoint& seam_point) const {
  // Pin x[0] == seam_point.x
  constraint_row_indices.push_back(constraint_count);
  constraint_col_indices.push_back(0);
  constraint_values.push_back(1.0);
  constraint_lower_bounds.push_back(seam_point.position.x);
  constraint_upper_bounds.push_back(seam_point.position.x);
  constraint_count++;

  // Pin y[0] == seam_point.y  (variable index 1)
  constraint_row_indices.push_back(constraint_count);
  constraint_col_indices.push_back(1);
  constraint_values.push_back(1.0);
  constraint_lower_bounds.push_back(seam_point.position.y);
  constraint_upper_bounds.push_back(seam_point.position.y);
  constraint_count++;
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
                                                        bool is_path_closed, bool is_final) const {
  if (center.size() != left.size() || center.size() != right.size() ||
      left.size() != center.size()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "The splines have different sizes. Right - %ld, Left - %ld, Center - %ld",
                center.size(), left.size(), right.size());
  }
  const int num_path_points = center.size();
  //TODA changethis is bad!!
  int total_points = num_path_points;

  if (num_path_points < 5) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Too few points for OSQP optimization (%d points). Minimum is 5.", num_path_points);
    return center;
  }

  if (is_final) {
    // ── Final iteration: optimize the full path as a closed loop ─────────────
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Final iteration: optimizing full path (%d points) as closed loop", total_points);
    return osqp_optimization_impl(center, left, right, /*opt_start=*/0, /*is_path_closed=*/true);
  }

  // ── Incremental iteration: sliding window over the last N points ──────────
  //TODA: change 50 to parameters
  const int window_size = std::min(50, total_points);
  const int opt_start = total_points - window_size;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "Incremental iteration: optimizing window [%d, %d) (%d points) as open path",
               opt_start, total_points, window_size);

  return osqp_optimization_impl(center, left, right, opt_start, /*is_path_closed=*/false);
}

std::vector<PathPoint> PathSmoothing::osqp_optimization_impl(const std::vector<PathPoint>& center,
                                                             const std::vector<PathPoint>& left,
                                                             const std::vector<PathPoint>& right,
                                                             int opt_start,
                                                             bool is_path_closed) const {
  const int total_points = static_cast<int>(center.size());

  if (center.size() != left.size() || center.size() != right.size()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "The splines have different sizes. Right - %ld, Left - %ld, Center - %ld",
                right.size(), left.size(), center.size());
  }

  // Slice inputs to the optimization window
  const std::vector<PathPoint> w_center(center.begin() + opt_start, center.end());
  const std::vector<PathPoint> w_left(left.begin() + opt_start, left.end());
  const std::vector<PathPoint> w_right(right.begin() + opt_start, right.end());

  const int num_path_points = static_cast<int>(w_center.size());

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
  std::vector<OSQPFloat> linear_objective(total_variables, 0.0);

  add_curvature_terms(num_path_points, circular_index, add_quadratic_coefficient, is_path_closed);
  add_slack_penalty_terms(num_path_points, add_quadratic_coefficient);
  add_proximity_terms(num_path_points, w_center, add_quadratic_coefficient, linear_objective);

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

  // ── Seam equality constraint ─────────────────────────────────────────────
  // When optimizing a window (not the full path), pin the first point of the
  // window to the last fixed point so the two segments stay connected.
  const bool has_fixed_prefix = (opt_start > 0);
  if (has_fixed_prefix) {
    add_seam_constraints(constraint_values, constraint_row_indices, constraint_col_indices,
                         constraint_lower_bounds, constraint_upper_bounds, constraint_count,
                         center[opt_start]);
  }

  add_boundary_constraints(constraint_values, constraint_row_indices, constraint_col_indices,
                           constraint_lower_bounds, constraint_upper_bounds, constraint_count,
                           w_left, w_right, num_path_points, safety_margin);

  add_slack_nonnegativity_constraints(constraint_values, constraint_row_indices,
                                      constraint_col_indices, constraint_lower_bounds,
                                      constraint_upper_bounds, constraint_count, num_path_points);

  const int total_constraints = constraint_count;

  std::vector<OSQPFloat> P_x;
  std::vector<OSQPInt> P_i, P_p;
  convert_to_csc_format(P_values, P_row_indices, P_col_indices, total_variables, P_x, P_i, P_p);

  std::vector<OSQPFloat> A_x;
  std::vector<OSQPInt> A_i, A_p;
  convert_to_csc_format(constraint_values, constraint_row_indices, constraint_col_indices,
                        total_variables, A_x, A_i, A_p);

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

  OSQPSettings solver_settings;
  ::osqp_set_default_settings(&solver_settings);
  solver_settings.verbose = false;
  solver_settings.max_iter = config_.max_iterations_;
  solver_settings.eps_abs = config_.tolerance_;
  solver_settings.eps_rel = config_.tolerance_;
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
  // Cache is keyed on window size. If the window size matches the previous
  // call we reuse primal/dual variables; otherwise we seed from center.
  if (!cached_primal_.empty() && cached_num_points_ == num_path_points) {
    std::vector<OSQPFloat> warm_x(total_variables, 0.0);
    std::vector<OSQPFloat> warm_y(total_constraints, 0.0);

    // Reuse full cached primal (same window size)
    for (int i = 0; i < num_path_points; ++i) {
      warm_x[2 * i] = cached_primal_[2 * i];
      warm_x[2 * i + 1] = cached_primal_[2 * i + 1];
    }

    // Reuse dual variables if constraint count also matches
    if (!cached_dual_.empty() && static_cast<int>(cached_dual_.size()) == total_constraints) {
      for (int i = 0; i < total_constraints; ++i) {
        warm_y[i] = cached_dual_[i];
      }
    }

    osqp_warm_start(solver_, warm_x.data(), warm_y.data());
  } else if (!cached_primal_.empty()) {
    // Window size changed (e.g. growing path before first full window):
    // seed from center coordinates, no dual reuse.
    std::vector<OSQPFloat> warm_x(total_variables, 0.0);
    std::vector<OSQPFloat> warm_y(total_constraints, 0.0);
    for (int i = 0; i < num_path_points; ++i) {
      warm_x[2 * i] = static_cast<OSQPFloat>(w_center[i].position.x);
      warm_x[2 * i + 1] = static_cast<OSQPFloat>(w_center[i].position.y);
    }
    osqp_warm_start(solver_, warm_x.data(), warm_y.data());
  }

  solve_status = ::osqp_solve(solver_);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OSQP status: %s | iterations: %lld | obj: %.4f",
              solver_->info->status, solver_->info->iter, solver_->info->obj_val);

  if (solver_->info->status_val != 1 && solver_->info->status_val != 2) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "OSQP did not converge (status: %s), returning original path",
                solver_->info->status);
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    return center;
  }

  if ((solver_->solution == nullptr) || (solver_->solution->x == nullptr)) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "OSQP solution is null, returning original path");
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    return center;
  }

  // ── Reassemble: fixed prefix + optimized window ───────────────────────────
  std::vector<PathPoint> result = center;  // copies fixed prefix unchanged
  for (int i = 0; i < num_path_points; ++i) {
    result[opt_start + i].position.x = solver_->solution->x[2 * i];
    result[opt_start + i].position.y = solver_->solution->x[2 * i + 1];
  }

  // Cache solution for next call (keyed on window size)
  cached_num_points_ = num_path_points;
  cached_is_closed_ = is_path_closed;
  cached_primal_.assign(solver_->solution->x, solver_->solution->x + total_variables);
  cached_dual_.assign(solver_->solution->y, solver_->solution->y + total_constraints);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "OSQP optimization completed: opt_start=%d, window=%d, total=%d (status: %lld)",
               opt_start, num_path_points, total_points, solve_status);

  return result;
}
