#include "planning/track.hpp"

#include <string>

#include "utils/color.hpp"

Track::Track() { completed = false; }

Track::~Track() {
  int lSize = leftCones.size();
  int rSize = rightCones.size();

  for (int i = 0; i < lSize; i++) delete leftCones[i];

  for (int i = 0; i < rSize; i++) delete rightCones[i];
}

void Track::fillTrack(const std::string &path) {
  std::string x, y, color;
  std::ifstream trackFile = openReadFile(path);
  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);
    addCone(xValue, yValue, color);
  }
  trackFile.close();
}

Cone *Track::getLeftConeAt(int index) { return leftCones[index]; }

Cone *Track::getRightConeAt(int index) { return rightCones[index]; }

int Track::getRightConesSize() { return rightCones.size(); }

int Track::getLeftConesSize() { return leftCones.size(); }

void Track::addCone(float xValue, float yValue, const std::string &color) {
  if (color == colors::color_names[colors::blue]) {
    // Blue Cones always have an even number id (2*x)
    rightCones.push_back(new Cone(this->rightCount * 2, xValue, yValue));
    rightCount++;
  } else if (color == colors::color_names[colors::yellow]) {
    // Yellow Cones always have an odd number id (2*x + 1)
    leftCones.push_back(new Cone(this->leftCount * 2 + 1, xValue, yValue));
    leftCount++;
  } else if (color != colors::color_names[colors::orange] &&
             color != colors::color_names[colors::large_orange]) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid color: %s\n", color.c_str());
  }
}

void Track::setCone(Cone *cone) {
  switch (cone->getId() % 2) {
    // todo binary search
    case colors::blue:
      for (size_t i = 0; i < leftCones.size(); i++) {
        if (leftCones[i]->getId() == cone->getId()) {
          leftCones[i]->setX(cone->getX());
          leftCones[i]->setY(cone->getY());
          return;
        }
      }
      leftCones.push_back(cone);
      break;
    case colors::yellow:
      for (size_t i = 0; i < rightCones.size(); i++) {
        if (rightCones[i]->getId() == cone->getId()) {
          rightCones[i]->setX(cone->getX());
          rightCones[i]->setY(cone->getY());
          return;
        }
      }
      rightCones.push_back(cone);
  }
}

Cone *Track::findCone(int id) {
  for (size_t i = 0; i < leftCones.size(); i++) {
    if (leftCones[i]->getId() == id) return leftCones[i];
  }

  for (size_t i = 0; i < rightCones.size(); i++) {
    if (rightCones[i]->getId() == id) return rightCones[i];
  }

  return nullptr;
}

Cone *Track::findCone(float x, float y) {
  for (size_t i = 0; i < leftCones.size(); i++) {
    if (leftCones[i]->getX() == x && leftCones[i]->getY() == y) return leftCones[i];
  }

  for (size_t i = 0; i < rightCones.size(); i++) {
    if (rightCones[i]->getX() == x && rightCones[i]->getY() == y) return rightCones[i];
  }

  return nullptr;
}

bool Track::vector_direction(Cone *c1, Cone *c2, float prev_vx, float prev_vy) {
  float vx = c2->getX() - c1->getX();
  float vy = c2->getY() - c1->getY();
  // Cos(angle) higher than 0 yhe vectors are pointing to the same half
  return (vx * prev_vx + vy * prev_vy) > 0;
}

int Track::validateCones() {
  int leftOutliers = deleteOutliers(1, 1.5, 3, 3, false);
  int rightOutliers = deleteOutliers(0, 1.5, 3, 3, false);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Deleted %i leftOutliers and %i right outliers\n",
              leftOutliers, rightOutliers);
  return leftOutliers + rightOutliers;
}

int Track::deleteOutliers(bool side, float distance_threshold, int order, float coeffs_ratio,
                          bool writing) {
  std::vector<Cone *> &unord_cone_seq = side ? leftCones : rightCones;
  // if side = 1(left) | = 0(right)

  const int n = unord_cone_seq.size();
  const int ncoeffs = n / coeffs_ratio;  // n > = ncoeffs
  const int nbreak = ncoeffs - order + 2;

  if (nbreak < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Too few points to calculate spline\n");
    return 0;
  }

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

  // Order cone_array
  // The algorithm works by iteratively selecting the nearest unvisited
  // cone to the current cone and updating the traversal direction accordingly.
  // This approach creates an ordered sequence that efficiently connects nearby
  // cones.
  std::vector<std::pair<Cone *, bool>> nn_unord_cone_seq;
  for (int i = 0; i < static_cast<int>(unord_cone_seq.size()); i++)
    nn_unord_cone_seq.push_back(std::make_pair(unord_cone_seq[i], false));

  // Values for first iteration
  std::vector<Cone *> cone_seq;
  Cone *cone1 = new Cone(-1, 0, 0);
  float vx = 1;
  float vy = 0;

  for (int iter_number = 0; iter_number < static_cast<int>(unord_cone_seq.size()); iter_number++) {
    float min_dist = MAXFLOAT;
    int min_index = 0;

    for (int i = 0; i < static_cast<int>(nn_unord_cone_seq.size()); i++) {
      Cone *cone2 = nn_unord_cone_seq[i].first;
      // Check only if not visited
      if (nn_unord_cone_seq[i].second == false ||
          (iter_number == 0 && vector_direction(cone1, cone2, vx, vy))) {
        // first iteration we assure the direction is correct to avoid going
        // backwards

        float new_dist = cone1->getDistanceTo(cone2);
        if (new_dist < min_dist) {
          min_dist = new_dist;
          min_index = i;
        }
      }
    }
    nn_unord_cone_seq[min_index].second = true;  // mark as visited

    // new visited cones is the reference for the next search. Arrays and Cone
    // updated
    vx = nn_unord_cone_seq[min_index].first->getX() - cone1->getX();
    vy = nn_unord_cone_seq[min_index].first->getY() - cone1->getY();
    cone1 = nn_unord_cone_seq[min_index].first;

    cone_seq.push_back(nn_unord_cone_seq[min_index].first);  // add cone to ordered sequence
  }

  // Set spline data
  for (int i = 0; i < n; i++) {
    gsl_vector_set(i_values, i, i);
    gsl_vector_set(x_values, i, cone_seq[i]->getX());
    gsl_vector_set(y_values, i, cone_seq[i]->getY());
    // closer cones more important(better stability)
    gsl_vector_set(w, i, 1.0 / pow(cone_seq[i]->getDistanceTo(cone_seq[(i + 1) % n]), 2));
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

  double xi, yi, yerr, yerr2;
  int divs = 10;
  std::vector<double> i_eval, x_eval, y_eval;
  std::vector<std::pair<double, double>> cone_seq_eval;  // Eval key_values

  // Output the smoothed curve for more desired points
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < divs; j += 1) {  // iterate over decimal numbers
      gsl_bspline_eval(i + static_cast<float>(j) / divs, B, bw);
      gsl_bspline_eval(i + static_cast<float>(j) / divs, C, cw);
      gsl_multifit_linear_est(B, c, cov, &xi, &yerr);
      gsl_multifit_linear_est(C, c2, cov2, &yi, &yerr2);
      i_eval.push_back(i);
      x_eval.push_back(xi);
      y_eval.push_back(yi);
      if (j == 0) {
        cone_seq_eval.push_back(std::make_pair(xi, yi));
        if (i == n - 1)  // Decimals can't go over last int
          break;
      }
    }
  }

  // Delete Outliers
  int outlierCount = 0;

  for (int i = 0; i < n; i++) {
    int index = i - outlierCount;  // decrease iterator indeleted indexes
    double dist = sqrt(pow(cone_seq[index]->getX() - cone_seq_eval[i].first, 2) +
                       pow(cone_seq[index]->getY() - cone_seq_eval[i].second, 2));
    if (dist > distance_threshold) {
      cone_seq.erase(cone_seq.begin() + index);
      outlierCount++;
    }
  }

  unord_cone_seq = cone_seq;

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

  if (writing) {
    // Write outputs in files
    std::string fileSide = side ? "1" : "0";

    std::ofstream splinePathFile = openWriteFile("src/planning/plots/spline" + fileSide + ".txt");

    for (int i = 0; i < static_cast<int>(i_eval.size()); i++)
      splinePathFile << x_eval[i] << " " << y_eval[i] << "\n";
    splinePathFile.close();

    std::ofstream outlierPathFile =
        openWriteFile("src/planning/plots/deletedoutliers" + fileSide + ".txt");
    for (int i = 0; i < static_cast<int>(cone_seq.size()); i++)
      outlierPathFile << cone_seq[i]->getX() << " " << cone_seq[i]->getY() << "\n";
    outlierPathFile.close();
  }

  return outlierCount;
}

void Track::reset() {
  leftCones.clear();
  rightCones.clear();
}