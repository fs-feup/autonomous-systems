#include "planning/smoothing.hpp"

std::vector<PathPoint> PathSmoothing::smooth_path(std::vector<PathPoint>& path,
                                                  bool is_path_closed) const {
  if (!config_.use_path_smoothing_) {
    return path;
  }
  if (is_path_closed) {
    path.push_back(path[0]);
  }

  const std::vector<PathPoint> spline_path = ::fit_spline(
      path, config_.spline_precision_, config_.spline_order_, config_.spline_coeffs_ratio_);

  return filter_path(spline_path);
}

std::vector<PathPoint> PathSmoothing::optimize_path(std::vector<PathPoint>& path,
                                                    const std::vector<PathPoint>& yellow_cones,
                                                    const std::vector<PathPoint>& blue_cones,
                                                    bool is_path_closed, bool is_final) {
  if (!config_.use_optimization_) {
    return smooth_path(path, is_path_closed);
  }

  const std::vector<PathPoint> optimized_path =
      osqp_optimization(path, blue_cones, yellow_cones, is_path_closed, is_final);

  return filter_path(optimized_path);
}

std::vector<PathPoint> PathSmoothing::filter_path(const std::vector<PathPoint>& path) const {
  std::vector<PathPoint> filtered_path;
  for (const auto& point : path) {
    if (filtered_path.empty() || filtered_path.back().position.euclidean_distance(point.position) >
                                     config_.min_path_point_distance_) {
      filtered_path.push_back(point);
    }
  }
  return filtered_path;
}

void PathSmoothing::add_curvature_terms(
    int num_path_points, const std::function<int(int)>& circular_index,
    const std::function<void(int, int, double)>& add_coefficient, bool is_path_closed) const {
  // For each point, we penalize: (p[i-1] - 2*p[i] + p[i+1])^2
  // OSQP minimizes (1/2) x^T P x, so all P coefficients must be multiplied by 2
  // to get the true mathematical penalty weight.

  int range_start = 1;
  int range_end = num_path_points - 1;

  if (is_path_closed) {
    range_start = 0;
    range_end = num_path_points;
  }

  for (int point_index = range_start; point_index < range_end; ++point_index) {
    const int prev_point_index = circular_index(point_index - 1);
    const int next_point_index = circular_index(point_index + 1);

    if (!is_path_closed && (prev_point_index == point_index || next_point_index == point_index)) {
      continue;
    }

    // X-coordinate curvature terms
    const int x_prev = 2 * prev_point_index;
    const int x_current = 2 * point_index;
    const int x_next = 2 * next_point_index;

    add_coefficient(x_prev, x_prev, 2 * config_.curvature_weight_);
    add_coefficient(x_current, x_current, 8 * config_.curvature_weight_);
    add_coefficient(x_next, x_next, 2 * config_.curvature_weight_);
    add_coefficient(x_prev, x_current, -4 * config_.curvature_weight_);
    add_coefficient(x_current, x_next, -4 * config_.curvature_weight_);
    add_coefficient(x_prev, x_next, 2 * config_.curvature_weight_);

    // Y-coordinate curvature terms
    const int y_prev = 2 * prev_point_index + 1;
    const int y_current = 2 * point_index + 1;
    const int y_next = 2 * next_point_index + 1;

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
  // Multiply by 2 to account for OSQP's (1/2) x^T P x objective scaling
  for (int slack_index = 0; slack_index < num_path_points; ++slack_index) {
    const int slack_variable_col = 2 * num_path_points + slack_index;
    add_coefficient(slack_variable_col, slack_variable_col, 2 * config_.safety_weight_);
  }
}

void PathSmoothing::add_proximity_terms(
    int num_path_points, const std::vector<PathPoint>& center_path,
    const std::function<void(int, int, double)>& add_quadratic_coefficient,
    std::vector<OSQPFloat>& linear_objective) const {
  // Penalize deviation from the center path to prevent degenerate solutions.

  for (int point_index = 0; point_index < num_path_points; ++point_index) {
    const int x_col = 2 * point_index;
    const int y_col = 2 * point_index + 1;

    add_quadratic_coefficient(x_col, x_col, 2 * config_.proximity_weight_);
    add_quadratic_coefficient(y_col, y_col, 2 * config_.proximity_weight_);

    linear_objective[x_col] =
        -2.0 * config_.proximity_weight_ * center_path[point_index].position.x;
    linear_objective[y_col] =
        -2.0 * config_.proximity_weight_ * center_path[point_index].position.y;
  }
}

void PathSmoothing::add_boundary_constraints(
    std::vector<OSQPFloat>& constraint_values, std::vector<OSQPInt>& constraint_row_indices,
    std::vector<OSQPInt>& constraint_col_indices, std::vector<OSQPFloat>& constraint_lower_bounds,
    std::vector<OSQPFloat>& constraint_upper_bounds, int& constraint_count,
    const std::vector<PathPoint>& left_boundary, const std::vector<PathPoint>& right_boundary,
    const std::vector<PathPoint>& center_path, int num_path_points, double safety_margin) const {
  Eigen::Vector2d previous_lateral_direction(0.0, 1.0);

  for (int point_index = 0; point_index < num_path_points; ++point_index) {
    const Eigen::Vector2d left_boundary_point(left_boundary[point_index].position.x,
                                              left_boundary[point_index].position.y);

    const Eigen::Vector2d right_boundary_point(right_boundary[point_index].position.x,
                                               right_boundary[point_index].position.y);

    // Compute forward direction from center path
    Eigen::Vector2d forward_direction;
    if (point_index + 1 < num_path_points) {
      forward_direction = Eigen::Vector2d(
          center_path[point_index + 1].position.x - center_path[point_index].position.x,
          center_path[point_index + 1].position.y - center_path[point_index].position.y);
    } else {
      forward_direction = Eigen::Vector2d(
          center_path[point_index].position.x - center_path[point_index - 1].position.x,
          center_path[point_index].position.y - center_path[point_index - 1].position.y);
    }

    // Reuse the previous direction: skipping left the point with no corridor constraint at all.
    Eigen::Vector2d lateral_direction;
    if (forward_direction.norm() < 1e-6) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Degenerate forward direction at point %d",
                  point_index);
      if (point_index == 0) {
        continue;
      }
      lateral_direction = previous_lateral_direction;
    } else {
      forward_direction.normalize();
      // Lateral is perpendicular to forward (points left by default)
      lateral_direction = Eigen::Vector2d(-forward_direction.y(), forward_direction.x());
    }

    // Ensure lateral points from right to left (left proj > right proj)
    double left_lateral_projection = left_boundary_point.dot(lateral_direction);
    double right_lateral_projection = right_boundary_point.dot(lateral_direction);

    if (left_lateral_projection < right_lateral_projection) {
      lateral_direction = -lateral_direction;
      left_lateral_projection = left_boundary_point.dot(lateral_direction);
      right_lateral_projection = right_boundary_point.dot(lateral_direction);
    }

    double right_bound = right_lateral_projection + safety_margin;
    double left_bound = left_lateral_projection - safety_margin;

    if (right_bound >= left_bound) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "Point %d: corridor too narrow (right=%.3f >= left=%.3f), clamping to midpoint",
                   point_index, right_bound, left_bound);
      const double midpoint = (right_bound + left_bound) / 2.0;
      right_bound = midpoint - 0.01;
      left_bound = midpoint + 0.01;
    }

    previous_lateral_direction = lateral_direction;

    const int slack_col = 2 * num_path_points + point_index;
    const int x_col = 2 * point_index;
    const int y_col = 2 * point_index + 1;

    // RIGHT constraint: lateral_direction · p + slack >= right_bound
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(x_col);
    constraint_values.push_back(lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(y_col);
    constraint_values.push_back(lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(slack_col);
    constraint_values.push_back(1.0);

    constraint_lower_bounds.push_back(right_bound);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;

    // LEFT constraint: lateral_direction · p - slack <= left_bound
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(x_col);
    constraint_values.push_back(lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(y_col);
    constraint_values.push_back(lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(slack_col);
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
  for (int slack_index = 0; slack_index < num_path_points; ++slack_index) {
    const int slack_col = 2 * num_path_points + slack_index;

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(slack_col);
    constraint_values.push_back(1.0);

    constraint_lower_bounds.push_back(0.0);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;
  }
}

void PathSmoothing::convert_to_csc_format(const std::vector<OSQPFloat>& values,
                                          const std::vector<OSQPInt>& row_indices,
                                          const std::vector<OSQPInt>& col_indices,
                                          int total_variables, std::vector<OSQPFloat>& csc_values,
                                          std::vector<OSQPInt>& csc_row_indices,
                                          std::vector<OSQPInt>& csc_col_pointers) const {
  // Group entries by column
  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> entries_per_column(total_variables);
  for (size_t entry_index = 0; entry_index < values.size(); ++entry_index) {
    entries_per_column[col_indices[entry_index]].push_back(
        {row_indices[entry_index], values[entry_index]});
  }

  // Sort each column by row index and merge duplicate entries
  for (int col = 0; col < total_variables; ++col) {
    std::sort(entries_per_column[col].begin(), entries_per_column[col].end(),
              [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

    std::vector<std::pair<OSQPInt, OSQPFloat>> merged_entries;
    for (const auto& [row, value] : entries_per_column[col]) {
      if (!merged_entries.empty() && merged_entries.back().first == row) {
        merged_entries.back().second += value;
      } else {
        merged_entries.push_back({row, value});
      }
    }
    entries_per_column[col] = std::move(merged_entries);
  }

  csc_values.reserve(values.size());
  csc_row_indices.reserve(values.size());
  csc_col_pointers.resize(total_variables + 1);

  csc_col_pointers[0] = 0;
  for (int col = 0; col < total_variables; ++col) {
    for (const auto& [row, value] : entries_per_column[col]) {
      csc_row_indices.push_back(row);
      csc_values.push_back(value);
    }
    csc_col_pointers[col + 1] = static_cast<OSQPInt>(csc_values.size());
  }
}

std::vector<PathPoint> PathSmoothing::osqp_optimization(
    const std::vector<PathPoint>& center_path, const std::vector<PathPoint>& left_boundary,
    const std::vector<PathPoint>& right_boundary, bool is_path_closed, bool is_final) const {
  if (center_path.size() != left_boundary.size() || center_path.size() != right_boundary.size()) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "Splines have different sizes. Right - %ld, Left - %ld, Center - %ld",
                right_boundary.size(), left_boundary.size(), center_path.size());
  }

  const int total_points = static_cast<int>(center_path.size());

  if (total_points < 5) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Too few points for OSQP optimization (%d points). Minimum is 5.", total_points);
    return center_path;
  }

  if (is_final) {
    cached_primal_.clear();
    cached_dual_.clear();
    cached_num_points_ = -1;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                "Final optimization with %d points.", total_points);
  }

  return osqp_optimization_implementation(center_path, left_boundary, right_boundary,
                                          is_path_closed);
}

void PathSmoothing::build_warm_start(int total_variables, int num_path_points,
                                     int total_constraints,
                                     const std::vector<PathPoint>& center_path) const {
  std::vector<OSQPFloat> warm_x(total_variables, 0.0);
  std::vector<OSQPFloat> warm_y(total_constraints, 0.0);

  if (!cached_primal_.empty() && cached_primal_.size() == static_cast<size_t>(total_variables)) {
    warm_x = cached_primal_;
  } else {
    for (int i = 0; i < num_path_points; ++i) {
      warm_x[2 * i] = static_cast<OSQPFloat>(center_path[i].position.x);
      warm_x[2 * i + 1] = static_cast<OSQPFloat>(center_path[i].position.y);
    }
  }

  if (!cached_dual_.empty() && cached_dual_.size() == static_cast<size_t>(total_constraints)) {
    warm_y = cached_dual_;
  }

  (void)::osqp_warm_start(solver_, warm_x.data(), warm_y.data());

}

std::vector<PathPoint> PathSmoothing::osqp_optimization_implementation(
    const std::vector<PathPoint>& center_path, const std::vector<PathPoint>& left_boundary,
    const std::vector<PathPoint>& right_boundary, bool is_path_closed) const {
  const int num_path_points = static_cast<int>(center_path.size());

  if (num_path_points < 5) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Too few points for OSQP (%d points). Minimum is 5.",
                num_path_points);
    return center_path;
  }

  const double safety_margin = config_.car_width_ / 2.0 + config_.safety_margin_;

  auto circular_index = [&](int index) {
    if (is_path_closed) {
      return (index + num_path_points) % num_path_points;
    }
    return std::clamp(index, 0, num_path_points - 1);
  };

  const int num_slack_variables = num_path_points;
  const int total_variables = 2 * num_path_points + num_slack_variables;

  // -------- BUILD QUADRATIC OBJECTIVE MATRIX (P) --------
  std::map<std::pair<int, int>, double> quadratic_terms;

  auto add_quadratic_coefficient = [&](int row_index, int col_index, double coefficient) {
    if (row_index > col_index) {
      std::swap(row_index, col_index);
    }
    quadratic_terms[{row_index, col_index}] += coefficient;
  };

  // -------- BUILD LINEAR OBJECTIVE VECTOR (q) --------
  std::vector<OSQPFloat> linear_objective(total_variables, 0.0);

  add_curvature_terms(num_path_points, circular_index, add_quadratic_coefficient, is_path_closed);
  add_slack_penalty_terms(num_path_points, add_quadratic_coefficient);
  add_proximity_terms(num_path_points, center_path, add_quadratic_coefficient, linear_objective);

  std::vector<OSQPFloat> P_values;
  std::vector<OSQPInt> P_row_indices, P_col_indices;
  P_values.reserve(quadratic_terms.size());
  P_row_indices.reserve(quadratic_terms.size());
  P_col_indices.reserve(quadratic_terms.size());

  for (const auto& [index_pair, coefficient] : quadratic_terms) {
    P_row_indices.push_back(index_pair.first);
    P_col_indices.push_back(index_pair.second);
    P_values.push_back(coefficient);
  }

  // -------- BUILD CONSTRAINT MATRIX (A) --------
  std::vector<OSQPFloat> constraint_values;
  std::vector<OSQPInt> constraint_row_indices, constraint_col_indices;
  std::vector<OSQPFloat> constraint_lower_bounds, constraint_upper_bounds;
  int constraint_count = 0;

  add_boundary_constraints(constraint_values, constraint_row_indices, constraint_col_indices,
                           constraint_lower_bounds, constraint_upper_bounds, constraint_count,
                           left_boundary, right_boundary, center_path, num_path_points,
                           safety_margin);

  add_slack_nonnegativity_constraints(constraint_values, constraint_row_indices,
                                      constraint_col_indices, constraint_lower_bounds,
                                      constraint_upper_bounds, constraint_count, num_path_points);

  const int total_constraints = constraint_count;

  // -------- CONVERT TO CSC FORMAT --------
  std::vector<OSQPFloat> P_csc_values;
  std::vector<OSQPInt> P_csc_row_indices, P_csc_col_pointers;
  convert_to_csc_format(P_values, P_row_indices, P_col_indices, total_variables, P_csc_values,
                        P_csc_row_indices, P_csc_col_pointers);

  std::vector<OSQPFloat> A_csc_values;
  std::vector<OSQPInt> A_csc_row_indices, A_csc_col_pointers;
  convert_to_csc_format(constraint_values, constraint_row_indices, constraint_col_indices,
                        total_variables, A_csc_values, A_csc_row_indices, A_csc_col_pointers);

  OSQPCscMatrix objective_matrix;
  objective_matrix.m = total_variables;
  objective_matrix.n = total_variables;
  objective_matrix.nzmax = static_cast<OSQPInt>(P_csc_values.size());
  objective_matrix.nz = -1;
  objective_matrix.x = P_csc_values.data();
  objective_matrix.i = P_csc_row_indices.data();
  objective_matrix.p = P_csc_col_pointers.data();

  OSQPCscMatrix constraint_matrix;
  constraint_matrix.m = total_constraints;
  constraint_matrix.n = total_variables;
  constraint_matrix.nzmax = static_cast<OSQPInt>(A_csc_values.size());
  constraint_matrix.nz = -1;
  constraint_matrix.x = A_csc_values.data();
  constraint_matrix.i = A_csc_row_indices.data();
  constraint_matrix.p = A_csc_col_pointers.data();

  // -------- SETUP OR UPDATE SOLVER --------
  if ((solver_ != nullptr) && (cached_num_points_ == num_path_points) &&
      (cached_is_closed_ == is_path_closed)) {
    (void)::osqp_update_data_mat(solver_, P_csc_values.data(), nullptr,
                                 static_cast<OSQPInt>(P_csc_values.size()), A_csc_values.data(),
                                 nullptr, static_cast<OSQPInt>(A_csc_values.size()));

    (void)::osqp_update_data_vec(solver_, linear_objective.data(), constraint_lower_bounds.data(),
                                 constraint_upper_bounds.data());
  }else{
    if (solver_ != nullptr) {
      (void)::osqp_cleanup(solver_);
      solver_ = nullptr;
    }

    OSQPSettings solver_settings;
    ::osqp_set_default_settings(&solver_settings);
    solver_settings.verbose = false;
    solver_settings.max_iter = config_.max_iterations_;
    solver_settings.eps_abs = config_.tolerance_;
    solver_settings.eps_rel = config_.tolerance_;
    solver_settings.polishing = 1;

    const OSQPInt setup_status =
        // osqp_setup takes the number of constraints (m) before the number of variables (n).
        ::osqp_setup(&solver_, &objective_matrix, linear_objective.data(), &constraint_matrix,
                     constraint_lower_bounds.data(), constraint_upper_bounds.data(),
                     total_constraints, total_variables, &solver_settings);

    if (setup_status != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OSQP setup failed with status %lld",
                   setup_status);
      (void)::osqp_cleanup(solver_);
      solver_ = nullptr;
      return center_path;
    }
  }

  // -------- WARM START --------
  build_warm_start(total_variables, num_path_points, total_constraints, center_path);

  const OSQPInt solve_status = ::osqp_solve(solver_);
  if (solve_status != 0) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "osqp_solve failed with status %lld", solve_status);
  }

  if (solver_->info->status_val != 1 && solver_->info->status_val != 2) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "OSQP did not converge (status: %s), returning original path",
                solver_->info->status);
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    return center_path;
  }

  if (solver_->solution == nullptr || solver_->solution->x == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "OSQP solution is null, returning original path");
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    return center_path;
  }

  std::vector<PathPoint> result_path = center_path;
  for (int i = 0; i < num_path_points; ++i) {
    result_path[i].position.x = solver_->solution->x[2 * i];
    result_path[i].position.y = solver_->solution->x[2 * i + 1];
  }

  // -------- UPDATE CACHE --------
  cached_num_points_ = num_path_points;
  cached_is_closed_ = is_path_closed;
  cached_primal_.assign(solver_->solution->x, solver_->solution->x + total_variables);
  cached_dual_.assign(solver_->solution->y, solver_->solution->y + total_constraints);

  return result_path;
}