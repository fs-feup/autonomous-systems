#ifndef SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_
#define SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_multifit.h>

#include <cmath>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/**
 * @brief In this file, a function is implemented to fit a spline to a set of ordered points.
 * The function, called 'fit_spline' is a template function, and to guarantee that it is only called
 * with suitable types, a series of static_asserts are used to check if the template type T has the
 * necessary methods defined. The structs below are used in the context of a method for static
 * assertion in C++17 called SFINAE ("Substitution Failure Is Not An Error").
 *
 * For each method that must be implemented, there are two structs with the same name: one that
 * inherits from false_type and another that inherits from true_type. When trying to instatiate a
 * struct, the compiler will choose the one that makes sense (if the method is implemented, the one
 * that inherits from true_type will be chosen; otherwise the one which inherits form false_type
 * will be used).
 *
 * After that, the compiler can assert at compile time if the type T implements each of the methods
 * that are necessary for the function to work properly with static_asserts (inside the "fit_spline"
 * function).
 *
 */

// Check for default constructor
template <typename T, typename = void>
struct HasDefaultConstructor : std::false_type {};

template <typename T>
struct HasDefaultConstructor<T, std::void_t<decltype(T())>> : std::true_type {};

// Check for hash function
template <typename T, typename = void>
struct IsHashable : std::false_type {};

template <typename T>
struct IsHashable<T, std::void_t<decltype(std::hash<T>{}(std::declval<T>()))>> : std::true_type {};

// Check for operator==
template <typename T, typename = void>
struct HasEqualityOperator : std::false_type {};

template <typename T>
struct HasEqualityOperator<T, std::void_t<decltype(std::declval<T>() == std::declval<T>())>>
    : std::true_type {};

// Check for position member
template <typename T, typename = void>
struct HasPosition : std::false_type {};

template <typename T>
struct HasPosition<T, std::void_t<decltype(std::declval<T>().position)>> : std::true_type {};

// Check for position.x and position.y members
template <typename T, typename = void>
struct HasPositionXY : std::false_type {};

template <typename T>
struct HasPositionXY<
    T, std::void_t<decltype(std::declval<T>().position.x), decltype(std::declval<T>().position.y)>>
    : std::true_type {};

// Check for copy constructor
template <typename T, typename = void>
struct IsCopyConstructor : std::false_type {};

template <typename T>
struct IsCopyConstructor<T, std::void_t<decltype(T(std::declval<T>()))>> : std::true_type {};

// Check for euclidean_distance method
template <typename T, typename = void>
struct HasEuclideanDistance : std::false_type {};

template <typename T>
struct HasEuclideanDistance<T, std::void_t<decltype(std::declval<T>().position.euclidean_distance(
                                   std::declval<T>().position))>> : std::true_type {};

// Check for position.x and position.y being double
template <typename T, typename = void>
struct PositionXYAreDouble : std::false_type {};

template <typename T>
struct PositionXYAreDouble<
    T, std::enable_if_t<std::is_same_v<decltype(std::declval<T>().position.x), double> &&
                        std::is_same_v<decltype(std::declval<T>().position.y), double>>>
    : std::true_type {};

/**
 * @brief This function takes a sequence of points (T), fits a spline to them using B-spline basis
 * functions, and returns the sequence of points that represent the fitted spline.
 *
 * @tparam T Type of the elements in the input and output sequences.
 *           T must satisfy several requirements:
 *           - Default constructible
 *           - Hashable
 *           - Equality comparable
 *           - Has a `position` member
 *           - `position` has `x` and `y` members of type `double`
 *           - `position` has a `euclidean_distance` method
 *
 * @param precision Number of interpolated points between each pair of original points.
 * @param order Order of the B-spline.
 * @param coeffs_ratio Ratio to determine the number of coefficients for the spline.
 * @param cone_seq Sequence of points to fit the spline to.
 * @return std::vector<T> Sequence of points representing the fitted spline.
 *
 * @note This function requires the GNU Scientific Library (GSL) for spline fitting.
 */
template <typename T>
std::vector<T> fit_spline(int precision, int order, float coeffs_ratio, std::vector<T> cone_seq) {
  static_assert(HasDefaultConstructor<T>::value, "T must be default constructible");
  static_assert(IsHashable<T>::value, "T must be hashable");
  static_assert(HasEqualityOperator<T>::value, "T must have operator==");
  static_assert(HasPosition<T>::value, "T must have a position member");
  static_assert(HasPositionXY<T>::value, "T.position must have x and y members");
  static_assert(IsCopyConstructor<T>::value, "T must be copyable");
  static_assert(HasEuclideanDistance<T>::value, "T.position must have a euclidean_distance method");
  static_assert(PositionXYAreDouble<T>::value, "T.position.x and T.position.y must be double");

  size_t n = cone_seq.size();
  if (n < 2) return cone_seq;

  // Garantee ncoeffs >= order -> mathematical requirement for B-splines
  size_t ncoeffs = static_cast<size_t>(static_cast<float>(n) / coeffs_ratio);
  if (ncoeffs < static_cast<size_t>(order)) return cone_seq;

  if (ncoeffs > n) ncoeffs = n;

  // Number of breakpoints in the B-spline (knots)
  const int nbreak = static_cast<int>(ncoeffs) - order + 2;

  if (nbreak < 2 || n == 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Too few points to calculate spline while executing 'fit_spline'"
                 "Number of cones was %i",
                 static_cast<int>(n));
    return cone_seq;
  }
  // -------- INITIALIZE GSL WORKSPACES --------
  gsl_bspline_workspace *bw =
      gsl_bspline_alloc(static_cast<size_t>(order), static_cast<size_t>(nbreak));
  gsl_bspline_workspace *cw = gsl_bspline_alloc(order, nbreak);

  gsl_vector *B = gsl_vector_alloc(ncoeffs);
  gsl_vector *C = gsl_vector_alloc(ncoeffs);
  gsl_vector *c = gsl_vector_alloc(ncoeffs);
  gsl_vector *c2 = gsl_vector_alloc(ncoeffs);

  gsl_vector *w = gsl_vector_alloc(n);
  gsl_vector *x_values = gsl_vector_alloc(n);
  gsl_vector *y_values = gsl_vector_alloc(n);
  gsl_vector *i_values = gsl_vector_alloc(n);

  gsl_matrix *X = gsl_matrix_alloc(n, ncoeffs);
  gsl_matrix *Y = gsl_matrix_alloc(n, ncoeffs);

  gsl_matrix *cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
  gsl_matrix *cov2 = gsl_matrix_alloc(ncoeffs, ncoeffs);

  gsl_multifit_linear_workspace *mw = gsl_multifit_linear_alloc(n, ncoeffs);
  gsl_multifit_linear_workspace *mw2 = gsl_multifit_linear_alloc(n, ncoeffs);

  double chisq, chisq2;

  // -------- FILL INPUT DATA AND WEIGHTS --------
  for (size_t i = 0; i < n; i++) {
    gsl_vector_set(i_values, i, static_cast<double>(i));
    gsl_vector_set(x_values, i, cone_seq[i].position.x);
    gsl_vector_set(y_values, i, cone_seq[i].position.y);

    // Weight points based on local spacing (closer points get larger weight)
    double dist_next = 1.0;
    if (i + 1 < n) {
      dist_next = cone_seq[i].position.euclidean_distance(cone_seq[i + 1].position);
    } else if (i > 0) {
      dist_next = cone_seq[i].position.euclidean_distance(cone_seq[i - 1].position);
    }
    if (dist_next < 1e-6) dist_next = 1e-6;

    gsl_vector_set(w, i, 1.0 / (dist_next * dist_next));
  }

  // -------- SET UNIFORM KNOTS FOR BOTH SPLINES --------
  double t_min = 0.0;
  double t_max = static_cast<double>(n - 1);
  gsl_bspline_knots_uniform(t_min, t_max, bw);
  gsl_bspline_knots_uniform(t_min, t_max, cw);

  // -------- BUILD DESIGN MATRICES --------
  // Evaluate each basis function at each input index
  for (int i = 0; i < static_cast<int>(n); i++) {
    gsl_bspline_eval(i, B, bw);
    gsl_bspline_eval(i, C, cw);

    // Fill design matrices for x and y least squares
    for (int j = 0; j < static_cast<int>(ncoeffs); j++) {
      gsl_matrix_set(X, i, j, gsl_vector_get(B, j));
      gsl_matrix_set(Y, i, j, gsl_vector_get(C, j));
    }
  }

  // -------- LEAST SQUARES FIT FOR X AND Y --------
  gsl_multifit_wlinear(X, w, x_values, c, cov, &chisq, mw);
  gsl_multifit_wlinear(Y, w, y_values, c2, cov2, &chisq2, mw2);

  // -------- EVALUATE THE SPLINE  --------
  std::vector<T> cone_seq_eval;
  cone_seq_eval.reserve(n * precision);
  for (int i = 0; i < static_cast<int>(n); i++) {
    for (int j = 0; j < precision; j++) {
      // Compute parameter t inside segment i
      double t = static_cast<double>(i) + static_cast<double>(j) / precision;
      if (t > t_max) t = t_max;

      gsl_bspline_eval(t, B, bw);
      gsl_bspline_eval(t, C, cw);

      // Compute x and y via estimated spline coefficients
      double xi, yi, err;
      gsl_multifit_linear_est(B, c, cov, &xi, &err);
      gsl_multifit_linear_est(C, c2, cov2, &yi, &err);

      // Create new spline point
      T new_element;
      new_element.position.x = xi;
      new_element.position.y = yi;
      cone_seq_eval.push_back(new_element);
    }
  }

  // -------- CLEANUP GSL RESOURCES --------
  gsl_bspline_free(bw);
  gsl_bspline_free(cw);
  gsl_vector_free(B);
  gsl_vector_free(C);
  gsl_vector_free(c);
  gsl_vector_free(c2);
  gsl_vector_free(w);
  gsl_vector_free(x_values);
  gsl_vector_free(y_values);
  gsl_vector_free(i_values);
  gsl_matrix_free(X);
  gsl_matrix_free(Y);
  gsl_matrix_free(cov);
  gsl_matrix_free(cov2);
  gsl_multifit_linear_free(mw);
  gsl_multifit_linear_free(mw2);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END fitSpline with %i points",
               static_cast<int>(cone_seq_eval.size()));

  return cone_seq_eval;
}

// WHERE SHOULD I PUT THIS STRUCTURE?
template <typename T>
struct TripleSpline {
  std::vector<T> center;
  std::vector<T> left;
  std::vector<T> right;
};

template <typename T>
TripleSpline<T> fitTripleSpline(const std::vector<T> &center, const std::vector<T> &left,
                                const std::vector<T> &right, int precision, int order,
                                float coeffs_ratio) {
  static_assert(HasDefaultConstructor<T>::value, "T must be default constructible");
  static_assert(IsHashable<T>::value, "T must be hashable");
  static_assert(HasEqualityOperator<T>::value, "T must have operator==");
  static_assert(HasPosition<T>::value, "T must have a position member");
  static_assert(HasPositionXY<T>::value, "T.position must have x and y members");
  static_assert(IsCopyConstructor<T>::value, "T must be copyable");
  static_assert(HasEuclideanDistance<T>::value, "T.position must have a euclidean_distance method");
  static_assert(PositionXYAreDouble<T>::value, "T.position.x and T.position.y must be double");

  TripleSpline<T> result;

  const size_t n = center.size();
  if (n < 2 || left.size() != n || right.size() != n) {
    return result;
  }

  // -------------------------
  // Number of spline coeffs
  // -------------------------
  size_t ncoeffs = static_cast<size_t>(static_cast<float>(n) / coeffs_ratio);
  if (ncoeffs < static_cast<size_t>(order)) {
    ncoeffs = order;
  }
  if (ncoeffs > n) {
    ncoeffs = n;
  }

  const int nbreak = static_cast<int>(ncoeffs) - order + 2;
  if (nbreak < 2) {
    return result;
  }

  const double t_min = 0.0;
  const double t_max = static_cast<double>(n - 1);

  // -------------------------
  // GSL workspaces
  // -------------------------
  gsl_bspline_workspace *bw = gsl_bspline_alloc(order, nbreak);

  gsl_vector *B = gsl_vector_alloc(ncoeffs);
  gsl_vector *w = gsl_vector_alloc(n);

  auto alloc_vec = [&]() { return gsl_vector_alloc(n); };
  auto alloc_mat = [&]() { return gsl_matrix_alloc(n, ncoeffs); };

  // Data vectors
  gsl_vector *cx = alloc_vec(), *cy = alloc_vec();
  gsl_vector *lx = alloc_vec(), *ly = alloc_vec();
  gsl_vector *rx = alloc_vec(), *ry = alloc_vec();

  gsl_matrix *X = alloc_mat();

  // Coefficients
  gsl_vector *ccx = gsl_vector_alloc(ncoeffs), *ccy = gsl_vector_alloc(ncoeffs);
  gsl_vector *clx = gsl_vector_alloc(ncoeffs), *cly = gsl_vector_alloc(ncoeffs);
  gsl_vector *crx = gsl_vector_alloc(ncoeffs), *cry = gsl_vector_alloc(ncoeffs);

  gsl_matrix *cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
  gsl_multifit_linear_workspace *mw = gsl_multifit_linear_alloc(n, ncoeffs);

  // -------------------------
  // Fill input data
  // -------------------------
  for (size_t i = 0; i < n; ++i) {
    gsl_vector_set(cx, i, center[i].position.x);
    gsl_vector_set(cy, i, center[i].position.y);
    gsl_vector_set(lx, i, left[i].position.x);
    gsl_vector_set(ly, i, left[i].position.y);
    gsl_vector_set(rx, i, right[i].position.x);
    gsl_vector_set(ry, i, right[i].position.y);

    // Distance-based weights (using center line spacing)
    double dist_next = 1.0;
    if (i + 1 < n) {
      dist_next = center[i].position.euclidean_distance(center[i + 1].position);
    } else if (i > 0) {
      dist_next = center[i].position.euclidean_distance(center[i - 1].position);
    }
    if (dist_next < 1e-6) dist_next = 1e-6;

    gsl_vector_set(w, i, 1.0 / (dist_next * dist_next));
  }

  // -------------------------
  // Uniform knots (same for all)
  // -------------------------
  gsl_bspline_knots_uniform(t_min, t_max, bw);

  // -------------------------
  // Design matrix
  // -------------------------
  for (size_t i = 0; i < n; ++i) {
    gsl_bspline_eval(static_cast<double>(i), B, bw);
    for (size_t j = 0; j < ncoeffs; ++j) gsl_matrix_set(X, i, j, gsl_vector_get(B, j));
  }

  double chisq;

  // -------------------------
  // Fit all splines
  // -------------------------
  auto fit = [&](gsl_vector *data, gsl_vector *coeffs) {
    gsl_multifit_wlinear(X, w, data, coeffs, cov, &chisq, mw);
  };

  fit(cx, ccx);
  fit(cy, ccy);
  fit(lx, clx);
  fit(ly, cly);
  fit(rx, crx);
  fit(ry, cry);

  // -------------------------
  // Evaluate splines
  // -------------------------
  const size_t total = n * precision;
  result.center.reserve(total);
  result.left.reserve(total);
  result.right.reserve(total);

  for (size_t i = 0; i < n; ++i) {
    for (int j = 0; j < precision; ++j) {
      double t = static_cast<double>(i) + static_cast<double>(j) / precision;
      if (t > t_max) t = t_max;

      gsl_bspline_eval(t, B, bw);

      auto eval = [&](gsl_vector *c, double &out) {
        double err;
        gsl_multifit_linear_est(B, c, cov, &out, &err);
      };

      T pc, pl, pr;

      eval(ccx, pc.position.x);
      eval(ccy, pc.position.y);
      eval(clx, pl.position.x);
      eval(cly, pl.position.y);
      eval(crx, pr.position.x);
      eval(cry, pr.position.y);

      result.center.push_back(pc);
      result.left.push_back(pl);
      result.right.push_back(pr);
    }
  }

  // -------------------------
  // Cleanup
  // -------------------------
  gsl_bspline_free(bw);
  gsl_vector_free(B);
  gsl_vector_free(w);

  gsl_vector_free(cx);
  gsl_vector_free(cy);
  gsl_vector_free(lx);
  gsl_vector_free(ly);
  gsl_vector_free(rx);
  gsl_vector_free(ry);

  gsl_vector_free(ccx);
  gsl_vector_free(ccy);
  gsl_vector_free(clx);
  gsl_vector_free(cly);
  gsl_vector_free(crx);
  gsl_vector_free(cry);

  gsl_matrix_free(X);
  gsl_matrix_free(cov);
  gsl_multifit_linear_free(mw);

  return result;
}

#endif  // SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_