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
                                                    std::vector<PathPoint>& yellow_cones,
                                                    std::vector<PathPoint>& blue_cones,
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

  if(is_path_closed){
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
  //TODA: Change this to param
  const double proximity_weight = 0.001 * config_.curvature_weight_;

  for (int point_index = 0; point_index < num_path_points; ++point_index) {
    const int x_col = 2 * point_index;
    const int y_col = 2 * point_index + 1;

    add_quadratic_coefficient(x_col, x_col, 2 * proximity_weight);
    add_quadratic_coefficient(y_col, y_col, 2 * proximity_weight);

    linear_objective[x_col] = -2.0 * proximity_weight * center_path[point_index].position.x;
    linear_objective[y_col] = -2.0 * proximity_weight * center_path[point_index].position.y;
  }
}

void PathSmoothing::add_boundary_constraints(
    std::vector<OSQPFloat>& constraint_values, std::vector<OSQPInt>& constraint_row_indices,
    std::vector<OSQPInt>& constraint_col_indices, std::vector<OSQPFloat>& constraint_lower_bounds,
    std::vector<OSQPFloat>& constraint_upper_bounds, int& constraint_count,
    const std::vector<PathPoint>& left_boundary, const std::vector<PathPoint>& right_boundary,
    const std::vector<PathPoint>& center_path, int num_path_points, double safety_margin) const {

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

    if (forward_direction.norm() < 1e-6) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Degenerate forward direction at point %d",
                  point_index);
      continue;
    }
    forward_direction.normalize();

    // Lateral is perpendicular to forward (points left by default)
    Eigen::Vector2d lateral_direction(-forward_direction.y(), forward_direction.x());

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
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Point %d: corridor too narrow (right=%.3f >= left=%.3f), clamping to midpoint",
                  point_index, right_bound, left_bound);
      const double midpoint = (right_bound + left_bound) / 2.0;
      right_bound = midpoint - 0.01;
      left_bound = midpoint + 0.01;
    }

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
    // Final lap: optimise the entire path as a closed loop.
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Final iteration: optimizing full path (%d points) closed", total_points);
    cached_primal_.clear();
    cached_dual_.clear();
    cached_num_points_ = -1;
    return osqp_optimization_impl(center_path, left_boundary, right_boundary,
                                  0, is_path_closed);
  }

  // TODA: Change 50 to a parameter
  const int window_size = std::min(50, total_points);
  const int window_start = total_points - window_size;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Incremental: window [%d, %d) (%d pts) open path",
               window_start, total_points, window_size);

  return osqp_optimization_impl(center_path, left_boundary, right_boundary, window_start,
                                is_path_closed);
}

std::vector<PathPoint> PathSmoothing::osqp_optimization_impl(
    const std::vector<PathPoint>& center_path, const std::vector<PathPoint>& left_boundary,
    const std::vector<PathPoint>& right_boundary, int window_start, bool is_path_closed) const {

  // Slice inputs to the sliding window
  const std::vector<PathPoint> window_center(center_path.begin() + window_start, center_path.end());
  const std::vector<PathPoint> window_left(left_boundary.begin() + window_start,
                                           left_boundary.end());
  const std::vector<PathPoint> window_right(right_boundary.begin() + window_start,
                                            right_boundary.end());

  const int num_path_points = static_cast<int>(window_center.size());

  if (num_path_points < 5) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Window too small for OSQP (%d points). Minimum is 5.", num_path_points);
    return center_path;
  }

  const double safety_margin = config_.car_width_ / 2.0 + config_.safety_margin_;

  auto circular_index = [&](int index) -> int {
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
    if (row_index > col_index) std::swap(row_index, col_index);
    quadratic_terms[{row_index, col_index}] += coefficient;
  };

  // -------- BUILD LINEAR OBJECTIVE VECTOR (q) --------
  std::vector<OSQPFloat> linear_objective(total_variables, 0.0);

  add_curvature_terms(num_path_points, circular_index, add_quadratic_coefficient, is_path_closed);
  add_slack_penalty_terms(num_path_points, add_quadratic_coefficient);
  add_proximity_terms(num_path_points, window_center, add_quadratic_coefficient, linear_objective);

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

  // Seam pinning via corridor tightening 
  std::vector<PathPoint> effective_left = window_left;
  std::vector<PathPoint> effective_right = window_right;

  if (window_start > 0 && globally_smoothed_path_.size() > static_cast<size_t>(window_start)) {
    const PathPoint& seam_point = globally_smoothed_path_[window_start];

    if (std::abs(seam_point.position.x) > 1000.0 || std::abs(seam_point.position.y) > 1000.0) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Seam corrupted (%.1f,%.1f), skipping",
                  seam_point.position.x, seam_point.position.y);
      globally_smoothed_path_.clear();
    } else {
      const double pin_half_width = 0.01;
      effective_left[0].position.x = seam_point.position.x + pin_half_width;
      effective_left[0].position.y = seam_point.position.y + pin_half_width;
      effective_right[0].position.x = seam_point.position.x - pin_half_width;
      effective_right[0].position.y = seam_point.position.y - pin_half_width;
    }
  }

  add_boundary_constraints(constraint_values, constraint_row_indices, constraint_col_indices,
                           constraint_lower_bounds, constraint_upper_bounds, constraint_count,
                           effective_left, effective_right, window_center, num_path_points,
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

  OSQPSettings solver_settings;
  ::osqp_set_default_settings(&solver_settings);
  solver_settings.verbose = true;
  solver_settings.max_iter = config_.max_iterations_;
  solver_settings.eps_abs = config_.tolerance_;
  solver_settings.eps_rel = config_.tolerance_;
  solver_settings.polishing = 1;

  if (solver_ != nullptr) {
    ::osqp_cleanup(solver_);
    solver_ = nullptr;
  }

  const OSQPInt setup_status =
      ::osqp_setup(&solver_, &objective_matrix, linear_objective.data(), &constraint_matrix,
                   constraint_lower_bounds.data(), constraint_upper_bounds.data(), total_variables,
                   total_constraints, &solver_settings);

  if (setup_status != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OSQP setup failed with status %lld", setup_status);
    ::osqp_cleanup(solver_);
    solver_ = nullptr;
    return center_path;
  }

  // -------- WARM START --------
  // TODA: ACTUALLY WARM STARTING!
  if (!cached_primal_.empty()) {
    std::vector<OSQPFloat> warm_start_primal(total_variables, 0.0);
    const bool same_window_size = (cached_num_points_ == num_path_points);
    const int window_shift = window_start - cached_opt_start_;

    if (same_window_size && window_start > 0) {
      if (window_shift > 0 && window_shift < num_path_points) {
        // Slide cached solution left: overlapping points carry over, new tail is seeded from center.
        for (int i = 0; i < num_path_points - window_shift; ++i) {
          warm_start_primal[2 * i] = cached_primal_[2 * (i + window_shift)];
          warm_start_primal[2 * i + 1] = cached_primal_[2 * (i + window_shift) + 1];
        }
        for (int i = num_path_points - window_shift; i < num_path_points; ++i) {
          warm_start_primal[2 * i] = static_cast<OSQPFloat>(window_center[i].position.x);
          warm_start_primal[2 * i + 1] = static_cast<OSQPFloat>(window_center[i].position.y);
        }
      } else if (window_shift == 0) {
        // Window didn't move: reuse the cached primal exactly.
        for (int i = 0; i < num_path_points; ++i) {
          warm_start_primal[2 * i] = cached_primal_[2 * i];
          warm_start_primal[2 * i + 1] = cached_primal_[2 * i + 1];
        }
      } else {
        // Shift is out of the useful range (negative or >= window): fall back to center path.
        for (int i = 0; i < num_path_points; ++i) {
          warm_start_primal[2 * i] = static_cast<OSQPFloat>(window_center[i].position.x);
          warm_start_primal[2 * i + 1] = static_cast<OSQPFloat>(window_center[i].position.y);
        }
      }
    } else {
      // Growing window or size mismatch: reuse what overlaps, seed the rest from center.
      const int reuse_count = std::min(cached_num_points_, num_path_points);
      for (int i = 0; i < reuse_count; ++i) {
        warm_start_primal[2 * i] = cached_primal_[2 * i];
        warm_start_primal[2 * i + 1] = cached_primal_[2 * i + 1];
      }
      for (int i = reuse_count; i < num_path_points; ++i) {
        warm_start_primal[2 * i] = static_cast<OSQPFloat>(window_center[i].position.x);
        warm_start_primal[2 * i + 1] = static_cast<OSQPFloat>(window_center[i].position.y);
      }
    }

    // Pin slot 0 to the globally-smoothed seam point so the solution is continuous
    // with the already-committed prefix. Prefer the smoothed value; fall back to
    // the raw center path if the smoothed path hasn't reached this index yet.
    if (window_start > 0 && !globally_smoothed_path_.empty() &&
        globally_smoothed_path_.size() > static_cast<size_t>(window_start)) {
      warm_start_primal[0] =
          static_cast<OSQPFloat>(globally_smoothed_path_[window_start].position.x);
      warm_start_primal[1] =
          static_cast<OSQPFloat>(globally_smoothed_path_[window_start].position.y);
    } else if (window_start > 0) {
      warm_start_primal[0] = static_cast<OSQPFloat>(center_path[window_start].position.x);
      warm_start_primal[1] = static_cast<OSQPFloat>(center_path[window_start].position.y);
    }

    osqp_warm_start(solver_, warm_start_primal.data(), nullptr);
  }

  const OSQPInt solve_status = ::osqp_solve(solver_);

  if (solver_->info->status_val != 1 && solver_->info->status_val != 2) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "OSQP did not converge (status: %s), returning original path",
                solver_->info->status);
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    globally_smoothed_path_.clear();
    return center_path;
  }

  if (solver_->solution == nullptr || solver_->solution->x == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "OSQP solution is null, returning original path");
    cached_num_points_ = -1;
    cached_primal_.clear();
    cached_dual_.clear();
    return center_path;
  }

  // Reassemble full result path 
  std::vector<PathPoint> result_path = center_path;

  // Overwrite the prefix with previously committed smoothed points.

  const int preserved_prefix_count =
      std::min(static_cast<int>(globally_smoothed_path_.size()), window_start);
  for (int i = 0; i < preserved_prefix_count; ++i) result_path[i] = globally_smoothed_path_[i];

  // OSQP uses: solution[2*i] = x, solution[2*i+1] = y for point i.
  for (int i = 0; i < num_path_points; ++i) {
    result_path[window_start + i].position.x = solver_->solution->x[2 * i];
    result_path[window_start + i].position.y = solver_->solution->x[2 * i + 1];
  }

  globally_smoothed_path_.assign(result_path.begin(),
                                 result_path.begin() + window_start + num_path_points);

  // Update cache
  cached_num_points_ = num_path_points;
  cached_opt_start_ = window_start;
  cached_is_closed_ = is_path_closed;
  cached_primal_.assign(solver_->solution->x, solver_->solution->x + total_variables);
  cached_dual_.assign(solver_->solution->y, solver_->solution->y + total_constraints);


  return result_path;
}