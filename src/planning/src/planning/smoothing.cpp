#include "planning/smoothing.hpp"

std::vector<PathPoint> PathSmoothing::smooth_path(std::vector<PathPoint>& path) const {
  if (!config_.use_path_smoothing_) {
    return path;
  }
  return fit_spline(path, config_.spline_precision_, config_.spline_order_, config_.spline_coeffs_ratio_);
}

std::vector<PathPoint> PathSmoothing::optimize_path(std::vector<PathPoint>& path,
                                                    std::vector<PathPoint>& yellow_cones,
                                                    std::vector<PathPoint>& blue_cones) const {
  if (!config_.use_path_smoothing_) {
    return path;
  }

  auto splines = fit_triple_spline(path, blue_cones, yellow_cones, config_.spline_precision_,
                                   config_.spline_order_, config_.spline_coeffs_ratio_);

  if (!config_.use_optimization_) {
    path = splines.center;
    return path;
  }

  return osqp_optimization(splines.center, splines.left, splines.right);
}

std::vector<PathPoint> PathSmoothing::osqp_optimization(const std::vector<PathPoint>& center,
                                                        const std::vector<PathPoint>& left,
                                                        const std::vector<PathPoint>& right) const {
  const int num_path_points = center.size();
  if (num_path_points < 5) {
    return center;
  }

  // NAO É NECESSARIO TER IGUAIS!
  const double curvature_weight = config_.curvature_weight_;
  const double smoothness_weight = config_.smoothness_weight_;
  const double safety_weight = config_.safety_weight_;
  const double safety_margin = config_.car_width_ / 2 + config_.safety_margin_;

  auto circular_index = [&](int i) { return (i + num_path_points) % num_path_points; };

  const int num_slack_variables = 2 * num_path_points;
  const int total_variables = 2 * num_path_points + num_slack_variables;

  std::map<std::pair<int, int>, double> quadratic_terms;

  auto add_quadratic_coefficient = [&](int row_idx, int col_idx, double coefficient) {
    if (row_idx > col_idx) {
      std::swap(row_idx, col_idx);
    }
    quadratic_terms[{row_idx, col_idx}] += coefficient;
  };

  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    int prev_point = circular_index(point_idx - 1);
    int next_point = circular_index(point_idx + 1);

    int x_prev = 2 * prev_point;
    int x_current = 2 * point_idx;
    int x_next = 2 * next_point;

    add_quadratic_coefficient(x_prev, x_prev, curvature_weight);
    add_quadratic_coefficient(x_current, x_current, 4 * curvature_weight);
    add_quadratic_coefficient(x_next, x_next, curvature_weight);
    add_quadratic_coefficient(x_prev, x_current, -2 * curvature_weight);
    add_quadratic_coefficient(x_current, x_next, -2 * curvature_weight);
    add_quadratic_coefficient(x_prev, x_next, curvature_weight);

    int y_prev = 2 * prev_point + 1;
    int y_current = 2 * point_idx + 1;
    int y_next = 2 * next_point + 1;

    add_quadratic_coefficient(y_prev, y_prev, curvature_weight);
    add_quadratic_coefficient(y_current, y_current, 4 * curvature_weight);
    add_quadratic_coefficient(y_next, y_next, curvature_weight);
    add_quadratic_coefficient(y_prev, y_current, -2 * curvature_weight);
    add_quadratic_coefficient(y_current, y_next, -2 * curvature_weight);
    add_quadratic_coefficient(y_prev, y_next, curvature_weight);
  }

  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    int next_point = circular_index(point_idx + 1);

    int x_current = 2 * point_idx;
    int x_next = 2 * next_point;
    add_quadratic_coefficient(x_current, x_current, smoothness_weight);
    add_quadratic_coefficient(x_next, x_next, smoothness_weight);
    add_quadratic_coefficient(x_current, x_next, -smoothness_weight);

    int y_current = 2 * point_idx + 1;
    int y_next = 2 * next_point + 1;
    add_quadratic_coefficient(y_current, y_current, smoothness_weight);
    add_quadratic_coefficient(y_next, y_next, smoothness_weight);
    add_quadratic_coefficient(y_current, y_next, -smoothness_weight);
  }

  for (int slack_idx = 0; slack_idx < num_slack_variables; ++slack_idx) {
    int slack_variable_index = 2 * num_path_points + slack_idx;
    add_quadratic_coefficient(slack_variable_index, slack_variable_index, safety_weight);
  }

  std::vector<OSQPFloat> linear_objective(total_variables, 0.0);

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

  for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
    Eigen::Vector2d left_boundary_point(left[point_idx].position.x, left[point_idx].position.y);
    Eigen::Vector2d right_boundary_point(right[point_idx].position.x, right[point_idx].position.y);
    Eigen::Vector2d lateral_direction = (left_boundary_point - right_boundary_point).normalized();

    double track_width = (left_boundary_point - right_boundary_point).norm();
    double adaptive_margin = std::max(safety_margin, 0.2 * track_width);

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx);
    constraint_values.push_back(lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx + 1);
    constraint_values.push_back(lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * num_path_points + 2 * point_idx);
    constraint_values.push_back(1.0);

    double right_boundary_constraint =
        right_boundary_point.dot(lateral_direction) + adaptive_margin;
    constraint_lower_bounds.push_back(right_boundary_constraint);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx);
    constraint_values.push_back(-lateral_direction.x());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * point_idx + 1);
    constraint_values.push_back(-lateral_direction.y());

    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * num_path_points + 2 * point_idx + 1);
    constraint_values.push_back(1.0);

    double left_boundary_constraint = -left_boundary_point.dot(lateral_direction) + adaptive_margin;
    constraint_lower_bounds.push_back(left_boundary_constraint);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;
  }

  for (int slack_idx = 0; slack_idx < num_slack_variables; ++slack_idx) {
    constraint_row_indices.push_back(constraint_count);
    constraint_col_indices.push_back(2 * num_path_points + slack_idx);
    constraint_values.push_back(1.0);

    constraint_lower_bounds.push_back(0.0);
    constraint_upper_bounds.push_back(OSQP_INFTY);
    constraint_count++;
  }

  const int total_constraints = constraint_count;

  OSQPCscMatrix* objective_matrix = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
  objective_matrix->m = total_variables;
  objective_matrix->n = total_variables;
  objective_matrix->nzmax = P_values.size();
  objective_matrix->nz = -1;
  objective_matrix->x = (OSQPFloat*)malloc(P_values.size() * sizeof(OSQPFloat));
  objective_matrix->i = (OSQPInt*)malloc(P_values.size() * sizeof(OSQPInt));
  objective_matrix->p = (OSQPInt*)malloc((total_variables + 1) * sizeof(OSQPInt));

  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> columns_P(total_variables);
  for (size_t entry = 0; entry < P_values.size(); ++entry) {
    columns_P[P_col_indices[entry]].push_back({P_row_indices[entry], P_values[entry]});
  }

  for (int col = 0; col < total_variables; ++col) {
    std::sort(columns_P[col].begin(), columns_P[col].end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
  }

  size_t csc_entry_index = 0;
  objective_matrix->p[0] = 0;
  for (int col = 0; col < total_variables; ++col) {
    for (const auto& [row, value] : columns_P[col]) {
      objective_matrix->i[csc_entry_index] = row;
      objective_matrix->x[csc_entry_index] = value;
      csc_entry_index++;
    }
    objective_matrix->p[col + 1] = csc_entry_index;
  }

  OSQPCscMatrix* constraint_matrix = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
  constraint_matrix->m = total_constraints;
  constraint_matrix->n = total_variables;
  constraint_matrix->nzmax = constraint_values.size();
  constraint_matrix->nz = -1;
  constraint_matrix->x = (OSQPFloat*)malloc(constraint_values.size() * sizeof(OSQPFloat));
  constraint_matrix->i = (OSQPInt*)malloc(constraint_values.size() * sizeof(OSQPInt));
  constraint_matrix->p = (OSQPInt*)malloc((total_variables + 1) * sizeof(OSQPInt));

  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> columns_A(total_variables);
  for (size_t entry = 0; entry < constraint_values.size(); ++entry) {
    columns_A[constraint_col_indices[entry]].push_back(
        {constraint_row_indices[entry], constraint_values[entry]});
  }

  for (int col = 0; col < total_variables; ++col) {
    std::sort(columns_A[col].begin(), columns_A[col].end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
  }

  csc_entry_index = 0;
  constraint_matrix->p[0] = 0;
  for (int col = 0; col < total_variables; ++col) {
    for (const auto& [row, value] : columns_A[col]) {
      constraint_matrix->i[csc_entry_index] = row;
      constraint_matrix->x[csc_entry_index] = value;
      csc_entry_index++;
    }
    constraint_matrix->p[col + 1] = csc_entry_index;
  }

  OSQPSettings* solver_settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
  osqp_set_default_settings(solver_settings);
  solver_settings->verbose = 1;
  solver_settings->max_iter = config_.max_iterations_;
  solver_settings->eps_abs = config_.tolerance_;
  solver_settings->eps_rel = config_.tolerance_;
  solver_settings->polishing = 1;

  OSQPSolver* solver = nullptr;

  OSQPInt setup_status =
      osqp_setup(&solver, objective_matrix, linear_objective.data(), constraint_matrix,
                 constraint_lower_bounds.data(), constraint_upper_bounds.data(), total_variables,
                 total_constraints, solver_settings);

  if (setup_status != 0) {
    if (solver) osqp_cleanup(solver);
    free(solver_settings);
    free(objective_matrix->x);
    free(objective_matrix->i);
    free(objective_matrix->p);
    free(objective_matrix);
    free(constraint_matrix->x);
    free(constraint_matrix->i);
    free(constraint_matrix->p);
    free(constraint_matrix);
    return center;
  }

  osqp_solve(solver);

  std::vector<PathPoint> optimized_path = center;

  if (solver->solution && solver->solution->x) {
    for (int point_idx = 0; point_idx < num_path_points; ++point_idx) {
      optimized_path[point_idx].position.x = solver->solution->x[2 * point_idx];
      optimized_path[point_idx].position.y = solver->solution->x[2 * point_idx + 1];
    }
  }

  osqp_cleanup(solver);
  free(solver_settings);
  free(objective_matrix->x);
  free(objective_matrix->i);
  free(objective_matrix->p);
  free(objective_matrix);
  free(constraint_matrix->x);
  free(constraint_matrix->i);
  free(constraint_matrix->p);
  free(constraint_matrix);

  return optimized_path;
}