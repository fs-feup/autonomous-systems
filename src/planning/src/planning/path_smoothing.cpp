#include "planning/path_smoothing.hpp"

PathSmoothing::PathSmoothing() {}

std::vector<PathPoint *> PathSmoothing::getPath() { return path; }

void PathSmoothing::addPathPoint(float xValue, float yValue, float vValue) {
  path.push_back(new PathPoint(xValue, yValue, vValue));
}

int PathSmoothing::getPointAmount() { return static_cast<int>(path.size()); }

void PathSmoothing::logPathPoints() {
  int n = getPointAmount();
  for (int i = 0; i < n; i++) {
    std::cout << "(" << path[i]->getX() << "," << path[i]->getY() << "),";
  }
  std::cout << std::endl;
}

void PathSmoothing::fillPath(const std::string &path) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "START fillPath");
  std::string x, y, v;
  std::ifstream trackFile = openReadFile(path);
  while (trackFile >> x >> y >> v) {
    float xValue = stof(x);
    float yValue = stof(y);
    float vValue = stof(v);
    addPathPoint(xValue, yValue, vValue);
  }
  trackFile.close();
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END fillPath");
}

bool PathSmoothing::vector_direction(PathPoint *c1, PathPoint *c2, float prev_vx, float prev_vy) {
  float vx = c2->getX() - c1->getX();
  float vy = c2->getY() - c1->getY();
  // Cos(angle) higher than 0 yhe vectors are pointing to the same half
  return (vx * prev_vx + vy * prev_vy) > 0;
}

std::vector<PathPoint *> PathSmoothing::orderPath(std::vector<PathPoint *> *unord_path) {
  // Order Pathpoint_array
  // The algorithm works by iteratively selecting the nearest unvisited
  // path_point to the current path_point and updating the traversal direction accordingly.
  // This approach creates an ordered sequence that efficiently connects nearby
  // path points.
  std::vector<std::pair<PathPoint *, bool>> new_unord_path;
  for (int i = 0; i < static_cast<int>(unord_path->size()); i++)
    new_unord_path.push_back(std::make_pair((*unord_path)[i], false));

  // Values for first iteration
  std::vector<PathPoint *> path;
  PathPoint *path_point1 = new PathPoint(0, 0, 0);
  float vx = 1;
  float vy = 0;

  for (int iter_number = 0; iter_number < static_cast<int>(unord_path->size()); iter_number++) {
    float min_dist = MAXFLOAT;
    int min_index = 0;

    for (int i = 0; i < static_cast<int>(new_unord_path.size()); i++) {
      PathPoint *path_point2 = new_unord_path[i].first;
      // Check only if not visited
      if (new_unord_path[i].second == false ||
          (iter_number == 0 && vector_direction(path_point1, path_point2, vx, vy))) {
        // first iteration we assure the direction is correct to avoid going
        // backwards

        float new_dist = path_point1->getDistanceTo(path_point2);
        if (new_dist < min_dist) {
          min_dist = new_dist;
          min_index = i;
        }
      }
    }
    new_unord_path[min_index].second = true;  // mark as visited

    // new visited path points is the reference for the next search. Arrays and path point
    // updated
    vx = new_unord_path[min_index].first->getX() - path_point1->getX();
    vy = new_unord_path[min_index].first->getY() - path_point1->getY();
    path_point1 = new_unord_path[min_index].first;

    path.push_back(new_unord_path[min_index].first);  // add path points to ordered sequence
  }
  return path;
}

void PathSmoothing::defaultSmoother(const std::vector<PathPoint *>& a_path) {
  path = pathSmoother(100, 3, 3, a_path);
}

std::vector<PathPoint *> PathSmoothing::splineSmoother(int precision, int order, float coeffs_ratio,
                                                       std::vector<PathPoint *> path) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "START splineSmoother with %i path points",
               static_cast<int>(path.size()));

  const int n = path.size();
  const int ncoeffs = n / coeffs_ratio;  // n > = ncoeffs
  const int nbreak = ncoeffs - order + 2;

  // Initialize vars (pointers)
  gsl_bspline_workspace *bw, *cw;
  gsl_vector *B, *C;
  gsl_vector *c, *c2, *w;
  gsl_vector *x_values, *y_values, *i_values;
  gsl_matrix *X, *Y, *cov, *cov2;
  gsl_multifit_linear_workspace *mw, *mw2;
  double chisq, chisq2;

  // allocate memory for the actual objects the pointers will point to
  bw = gsl_bspline_alloc(order, nbreak);
  cw = gsl_bspline_alloc(order, nbreak);
  B = gsl_vector_alloc(ncoeffs);
  C = gsl_vector_alloc(ncoeffs);
  i_values = gsl_vector_alloc(n);
  x_values = gsl_vector_alloc(n);
  y_values = gsl_vector_alloc(n);
  X = gsl_matrix_alloc(n, ncoeffs);
  Y = gsl_matrix_alloc(n, ncoeffs);
  c = gsl_vector_alloc(ncoeffs);
  c2 = gsl_vector_alloc(ncoeffs);
  w = gsl_vector_alloc(n);
  cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
  cov2 = gsl_matrix_alloc(ncoeffs, ncoeffs);
  mw = gsl_multifit_linear_alloc(n, ncoeffs);
  mw2 = gsl_multifit_linear_alloc(n, ncoeffs);

  // Set spline data
  for (int i = 0; i < n; i++) {
    gsl_vector_set(i_values, i, i);
    gsl_vector_set(x_values, i, path[i]->getX());
    gsl_vector_set(y_values, i, path[i]->getY());
    // closer cones more important(better stability)
    gsl_vector_set(w, i, 1.0 / pow(path[i]->getDistanceTo(path[(i + 1) % n]), 2));
  }

  // Set i range within cone set length
  gsl_bspline_knots_uniform(0, n, bw);
  gsl_bspline_knots_uniform(0, n, cw);

  /* construct the fit matrix X */
  for (int i = 0; i < n; i++) {
    /* compute B_j(xi) for all j */
    gsl_bspline_eval(i, B, bw);
    gsl_bspline_eval(i, C, cw);

    /* fill in row i of X */
    for (int j = 0; j < ncoeffs; j++) {
      double Bj = gsl_vector_get(B, j);
      gsl_matrix_set(X, i, j, Bj);
      double Cj = gsl_vector_get(C, j);
      gsl_matrix_set(Y, i, j, Cj);
    }
  }

  // Fit spline
  gsl_multifit_wlinear(X, w, x_values, c, cov, &chisq, mw);
  gsl_multifit_wlinear(Y, w, y_values, c2, cov2, &chisq2, mw2);

  // Initialize variables for subsequent spline evaluation
  double xi, yi, yerr, yerr2;
  std::vector<double> i_eval, x_eval, y_eval;
  std::vector<PathPoint *> path_eval;

  // Calculate the desired amount of spline points and add them to "path_eval"
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < precision; j += 1) {  // iterate over decimal numbers
      gsl_bspline_eval(i + static_cast<float>(j) / precision, B, bw);
      gsl_bspline_eval(i + static_cast<float>(j) / precision, C, cw);
      gsl_multifit_linear_est(B, c, cov, &xi, &yerr);
      gsl_multifit_linear_est(C, c2, cov2, &yi, &yerr2);
      i_eval.push_back(i);
      x_eval.push_back(xi);
      y_eval.push_back(yi);
      PathPoint *path_point = new PathPoint(xi, yi);
      path_eval.push_back(path_point);
      if (j == 0 && i == n - 1) {
        break;  // Decimals can't go over last int
      }
    }
  }

  // Free Memory
  gsl_bspline_free(bw);
  gsl_bspline_free(cw);
  gsl_vector_free(B);
  gsl_vector_free(C);
  gsl_vector_free(i_values);
  gsl_vector_free(x_values);
  gsl_vector_free(y_values);
  gsl_matrix_free(X);
  gsl_matrix_free(Y);
  gsl_vector_free(c);
  gsl_vector_free(w);
  gsl_matrix_free(cov);
  gsl_multifit_linear_free(mw);
  gsl_multifit_linear_free(mw2);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END fitSpline with %i points",
               static_cast<int>(path_eval.size()));

  return path_eval;
}

std::vector<PathPoint *> PathSmoothing::pathSmoother(int precision, int order, float coeffs_ratio,
                                                     std::vector<PathPoint *> unord_path) {
  std::vector<PathPoint *> ord_path = orderPath(&unord_path);
  std::vector<PathPoint *> path = splineSmoother(precision, order, coeffs_ratio, ord_path);
  return path;
}
