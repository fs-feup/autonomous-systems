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

void Track::fillTrack(const std::string& path) {
  std::string x, y, color;
  std::ifstream trackFile;

  trackFile.open(path);

  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);

    addCone(xValue, yValue, color);
  }
}

Cone* Track::getLeftConeAt(int index) { return leftCones[index]; }

Cone* Track::getRightConeAt(int index) { return rightCones[index]; }

int Track::getRightConesSize() { return rightCones.size(); }

int Track::getLeftConesSize() { return leftCones.size(); }

void Track::addCone(float xValue, float yValue, const std::string& color) {
  if (color == colors::color_names[colors::blue]) {
    rightCones.push_back(new Cone(this->rightCount * 2, xValue, yValue));
    rightCount++;
  } else if (color == colors::color_names[colors::yellow]) {
    leftCones.push_back(new Cone(this->leftCount * 2 + 1, xValue, yValue));
    leftCount++;
  } else if (color != colors::color_names[colors::orange] &&
             color != colors::color_names[colors::large_orange]) {
    std::cout << "Invalid color: " << color << std::endl;
  }
}

void Track::setCone(Cone* cone) {
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

Cone* Track::findCone(int id) {
  for (size_t i = 0; i < leftCones.size(); i++) {
    if (leftCones[i]->getId() == id) return leftCones[i];
  }

  for (size_t i = 0; i < rightCones.size(); i++) {
    if (rightCones[i]->getId() == id) return rightCones[i];
  }

  return nullptr;
}

Cone* Track::findCone(float x, float y) {
  for (size_t i = 0; i < leftCones.size(); i++) {
    if (leftCones[i]->getX() == x && leftCones[i]->getY() == y) return leftCones[i];
  }

  for (size_t i = 0; i < rightCones.size(); i++) {
    if (rightCones[i]->getX() == x && rightCones[i]->getY() == y) return rightCones[i];
  }

  return nullptr;
}

void Track::deleteOutliers(float distance_threshold){
  std::vector<Cone> outliers;

  std::cout << "Outlier removal\n";

  const int order = 3;
  const size_t n = leftCones.size();
  const size_t ncoeffs = (n >= 47 ? 47 : n / 2); // n > = ncoeffs
  const size_t nbreak = ncoeffs - order + 2;
  
  gsl_bspline_workspace *bw, *cw;
  gsl_vector *B, *C;
  gsl_vector *c, *c2, *w;
  gsl_vector *x_values, *y_values, *i_values;
  gsl_matrix *X, *Y, *cov, *cov2;
  gsl_multifit_linear_workspace *mw, *mw2;
  double chisq, chisq2;

  /* allocate a cubic bspline workspace (k = order) */
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

  /* this is the data to be fitted */
  for (size_t i = 0; i < n; i++)
    {
      gsl_vector_set(i_values, i, i);
      gsl_vector_set(x_values, i, leftCones[i]->getX());
      gsl_vector_set(y_values, i, leftCones[i]->getY());
      gsl_vector_set(w, i, 1.0 / pow(leftCones[i]->getDistanceTo(leftCones[(i + 1) % n]), 2));
    }

  /* use uniform breakpoints on [0, n] */
  gsl_bspline_knots_uniform(0, n, bw);
  gsl_bspline_knots_uniform(0, n, cw);

  /* construct the fit matrix X */
  for (size_t i = 0; i < n; i++)
    {
      /* compute B_j(xi) for all j */
      gsl_bspline_eval(i, B, bw);
      gsl_bspline_eval(i, C, cw);

      /* fill in row i of X */
      for (size_t j = 0; j < ncoeffs; j++)
        {
          double Bj = gsl_vector_get(B, j);
          gsl_matrix_set(X, i, j, Bj);
          double Cj = gsl_vector_get(C, j);
          gsl_matrix_set(Y, i, j, Cj);
        }
    }

  /* do the fit */
  gsl_multifit_wlinear(X, w, x_values, c, cov, &chisq, mw);
  gsl_multifit_wlinear(Y, w, y_values, c2, cov2, &chisq2, mw2);

  /* output the smoothed curve */
  
  double xi, yi, yerr, yerr2;
  std::vector<double> i_eval, x_eval, y_eval;

  for (float is = 0.0; is <= n - 1; is += 0.1)
    {
      gsl_bspline_eval(is, B, bw);
      gsl_bspline_eval(is, C, cw);
      gsl_multifit_linear_est(B, c, cov, &xi, &yerr);
      gsl_multifit_linear_est(C, c2, cov2, &yi, &yerr2);
      if (xi > 40 && yi < -20) std::cout << is << " " << xi << "\n";
      i_eval.push_back(is);
      x_eval.push_back(xi);
      y_eval.push_back(yi);
    }
  

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

    std::string filePrefix = rcpputils::fs::current_path().string();
    std::string finalPath = filePrefix + "/planning/planning/tracks/spline.txt";
    std::ofstream finalPathFile(finalPath);
    for (size_t i = 0; i < i_eval.size(); i++)
      finalPathFile << x_eval[i] << " " << y_eval[i] << "\n";
    finalPathFile.close();

}

void Track::reset() {
  leftCones.clear();
  rightCones.clear();
}
