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

void Track::logCones(bool side) {
  if (side) {
    std::cout << "LEFT CONES: ";
    for (int t = 0; t < static_cast<int>(leftCones.size()); t++) {
      std::cout << "(" << leftCones[t] -> getX() << "," << leftCones[t] -> getY() << "),";
    }
  } else {
    std::cout << "RIGHT CONES: ";
    for (int t = 0; t < static_cast<int>(rightCones.size()); t++) {
      std::cout << "(" << rightCones[t] -> getX() << "," << rightCones[t] -> getY() << "),";
    }
  }
  std::cout << std::endl;
}

float Track::round_n(float num, int decimal_places) {
  num *= pow(10, decimal_places);
  int intermediate = round(num);
  num = intermediate/pow(10, decimal_places);
  return num;
}
float Track::getMaxDistance(bool side) {
  float distance = 0;
  if (side) {
    int n = leftCones.size();
    Cone *cone1 = leftCones[0];
    for (int i = 1; i < n; i++) {
      Cone *cone2 = leftCones[1];
      float dist = sqrt(pow(cone1 -> getX() - cone2 -> getX(), 2)
      +pow(cone1 -> getY() - cone2 -> getY(), 2));
      if (dist > distance) {
        distance =  dist;
      }
      cone1 = cone2;
    }
  } else {
    int n = rightCones.size();
    Cone *cone1 = rightCones[0];
    for (int i = 1; i < n; i++) {
      Cone *cone2 = rightCones[1];
      float dist = sqrt(pow(cone1 -> getX() - cone2 -> getX(), 2)+
      pow(cone1 -> getY() - cone2 -> getY(), 2));
      if (dist > distance) {
        distance =  dist;
      }
      cone1 = cone2;
    }
  }
  return distance;
}

void Track::fillTrack(const std::string &path) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "START fillTrack");
  std::string x, y, color;
  std::ifstream trackFile = openReadFile(path);
  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);
    addCone(xValue, yValue, color);
  }
  trackFile.close();
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END fillTrack");
}

Cone *Track::getLeftConeAt(int index) { return leftCones[index]; }

Cone *Track::getRightConeAt(int index) { return rightCones[index]; }

int Track::getRightConesSize() { return rightCones.size(); }

int Track::getLeftConesSize() { return leftCones.size(); }

void Track::addCone(float xValue, float yValue, const std::string &color) {
  if (color == colors::color_names[colors::blue]) {
    // Blue Cones always have an even number id (2*x)
    leftCones.push_back(new Cone(this->leftCount * 2, xValue, yValue));
    leftCount++;
  } else if (color == colors::color_names[colors::yellow]) {
    // Yellow Cones always have an odd number id (2*x + 1)
    rightCones.push_back(new Cone(this->rightCount * 2 + 1, xValue, yValue));
    rightCount++;
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

std::string Track::gsl_vectorLog(gsl_vector *gsl, int n) {
  std::string acc = "";
    double *it = gsl->data;
  for (int t=0; t < n; t++) {
    acc += std::to_string(*it);
    acc += " ";
    it++;
  }
  return acc;
}

std::vector<Cone *> Track::orderCones(std::vector<Cone *> *unord_cone_seq) {
  // Order cone_array
  // The algorithm works by iteratively selecting the nearest unvisited
  // cone to the current cone and updating the traversal direction accordingly.
  // This approach creates an ordered sequence that efficiently connects nearby
  // cones.
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "START orderCones");
  std::vector<std::pair<Cone *, bool>> nn_unord_cone_seq;
  for (int i = 0; i < static_cast<int>(unord_cone_seq -> size()); i++)
    nn_unord_cone_seq.push_back(std::make_pair((*unord_cone_seq)[i], false));

  // Values for first iteration
  std::vector<Cone *> cone_seq;
  Cone *cone1 = new Cone(-1, 0, 0);
  float vx = 1;
  float vy = 0;

  for (int iter_number = 0; iter_number < static_cast<int>(unord_cone_seq->size());
       iter_number++) {
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
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END orderCones");
  return cone_seq;
}



std::vector<Cone *> Track::fitSpline(bool side, int precision, int order,
 float coeffs_ratio,  std::vector<Cone *> cone_seq) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
  "START fitSpline with %i cones", static_cast<int>(cone_seq.size()));

  //START ELIMINATING OUTLIERS BASED ON DISTANCE TO SPLINE
  const int n = cone_seq.size();
  const int ncoeffs = n / coeffs_ratio;  // n > = ncoeffs
  const int nbreak = ncoeffs - order + 2;

  if (n == 0) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
    "Number of cones is 0 while executing 'deleteOutliers' on side %i", side);
  }
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
  "nbreak= %i; n (number of cones) = %i; ncoeffs = %i; left cones: %i; right cones: %i\n",
    nbreak, n, ncoeffs,  static_cast<int>(leftCones.size()),  static_cast<int>(rightCones.size()));
  if (nbreak < 2) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
    "Too few points to calculate spline on side %i", side);
    Cone *n_cone =  new Cone(0, 0, 0);
    return {n_cone};
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

  //Log spline coefficients held in gsl_vectors c and c2
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Base functions' weights in x: %s\n",
  gsl_vectorLog(c, ncoeffs).c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Base functions' weights in y: %s\n",
  gsl_vectorLog(c2, ncoeffs).c_str());

  //Initialize variables for subsequent spline evaluation
  double xi, yi, yerr, yerr2;
  std::vector<double> i_eval, x_eval, y_eval;
  std::vector<Cone *> cone_seq_eval;

  // Calculate the desired amount of spline points and add them to "cone_seq_eval"
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < precision; j += 1) {  // iterate over decimal numbers
      gsl_bspline_eval(i + static_cast<float>(j) / precision, B, bw);
      gsl_bspline_eval(i + static_cast<float>(j) / precision, C, cw);
      gsl_multifit_linear_est(B, c, cov, &xi, &yerr);
      gsl_multifit_linear_est(C, c2, cov2, &yi, &yerr2);
      i_eval.push_back(i);
      x_eval.push_back(xi);
      y_eval.push_back(yi);
      int id = static_cast<int>(cone_seq_eval.size()) + 1 - side;
      Cone *cone_new = new Cone(id, xi, yi);
      cone_seq_eval.push_back(cone_new);
      if (j == 0 && i == n - 1) {
        break; // Decimals can't go over last int
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
  static_cast<int>(cone_seq_eval.size()));

  // To access spline points uncomment these lines
  // for (int i=0; i<static_cast<int>(cone_seq_eval.size()); i++){
  //   std::cout << "(" <<  cone_seq_eval[i] -> getX() << "," << cone_seq_eval[i] -> getY() << "),";
  // }
  // std::cout <<std::endl;

  return cone_seq_eval;
}

void Track::validateCones() {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "START validateCones\n");

  deleteOutliers(1, 3, 3, false, 1);
  deleteOutliers(0, 3, 3, false, 1);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END validateCones\n");
}

void Track::deleteOutliers(bool side, int order, float coeffs_ratio,
                          bool writing, int precision) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "START deleteOutliers\n");
  std::vector<Cone *> &unord_cone_seq = side ? leftCones : rightCones;
  // if side = 1(left) | = 0(right)


  // Order cones by going from each cone to the neearest unvisited one
  // This is necessary since splines are order dependent
  std::vector<Cone *> cone_seq  = orderCones(&unord_cone_seq);


  //Calculate spline points that fit the sequence of cones
  std::vector<Cone *> cone_seq_eval = fitSpline(side, precision, order,
  coeffs_ratio, cone_seq);

  //ELIMINATE CONES BASED ON DISTANCE TO CORRESPONDING SPLINE POINT
  //CURRENTLY NOT USED
  /*int outlierCount = 0;
  for (int i = 0; i < static_cast<int>(unord_cone_seq.size()); i++) {
    int index = i - outlierCount;  // decrease iterator indeleted indexes
    double dist = sqrt(pow(cone_seq[index]->getX() - cone_seq_eval[i*precision]->getX(), 2) +
                       pow(cone_seq[index]->getY() - cone_seq_eval[i*precision]->getY(), 2));
    if (dist > distance_threshold) {
      Cone *deleted_cone = *(cone_seq.begin() + index);
      double x_erased = deleted_cone -> getX();
      double y_erased = deleted_cone -> getY();
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Outlier removed at x= %f and y= %f on side %i\n", x_erased, y_erased, side);
      cone_seq.erase(cone_seq.begin() + index);
      outlierCount++;
    }
  }*/
  // SMOOTH CONES TO AVOID HAVING TOO MUCH SPACE BETWEEN CONES
  // cone_seq = fitSpline(side, 1, 3, 3, cone_seq);
  // END ELIMINATING OUTLIERS BASED ON DISTANCE TO CORRESPONDING SPLINE POINT

  //OPTIONALLY WRITE SPLINE POINTS TO FILE
  if (writing) {
    std::string fileSide = side ? "1" : "0";
    std::ofstream splinePathFile = openWriteFile("src/planning/plots/spline" + fileSide + ".txt");

    for (int i = 0; i < static_cast<int>(cone_seq_eval.size()); i++)
      splinePathFile <<"(" << cone_seq_eval[i]->getX() << ", " << cone_seq_eval[i]->getY() << ")\n";
    splinePathFile.close();
  }
  //STOP WRITING OUTPUT TO FILE

  unord_cone_seq = cone_seq_eval;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END deleteOutliers\n");
}

void Track::reset() {
  leftCones.clear();
  rightCones.clear();
}