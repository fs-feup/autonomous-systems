#ifndef SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_
#define SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_spline.h>

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
struct HasDefaultConstructor : public std::false_type {};

template <typename T>
struct HasDefaultConstructor<T, std::void_t<decltype(T())>> : public std::true_type {};

// Check for hash function
template <typename T, typename = void>
struct IsHashable : public std::false_type {};

template <typename T>
struct IsHashable<T, std::void_t<decltype(std::hash<T>{}(std::declval<T>()))>>
    : public std::true_type {};

// Check for operator==
template <typename T, typename = void>
struct HasEqualityOperator : public std::false_type {};

template <typename T>
struct HasEqualityOperator<T, std::void_t<decltype(std::declval<T>() == std::declval<T>())>>
    : public std::true_type {};

// Check for position member
template <typename T, typename = void>
struct HasPosition : public std::false_type {};

template <typename T>
struct HasPosition<T, std::void_t<decltype(std::declval<T>().position)>> : public std::true_type {};

// Check for position.x and position.y members
template <typename T, typename = void>
struct HasPositionXY : public std::false_type {};

template <typename T>
struct HasPositionXY<
    T, std::void_t<decltype(std::declval<T>().position.x), decltype(std::declval<T>().position.y)>>
    : public std::true_type {};

// Check for copy constructor
template <typename T, typename = void>
struct IsCopyConstructor : public std::false_type {};

template <typename T>
struct IsCopyConstructor<T, std::void_t<decltype(T(std::declval<T>()))>> : public std::true_type {};

// Check for euclidean_distance method
template <typename T, typename = void>
struct HasEuclideanDistance : public std::false_type {};

template <typename T>
struct HasEuclideanDistance<T, std::void_t<decltype(std::declval<T>().position.euclidean_distance(
                                   std::declval<T>().position))>> : public std::true_type {};

// Check for position.x and position.y being double
template <typename T, typename = void>
struct PositionXYAreDouble : public std::false_type {};

template <typename T>
struct TripleSpline {
  std::vector<T> center;
  std::vector<T> left;
  std::vector<T> right;
};

template <typename T>
struct PositionXYAreDouble<
    T, std::enable_if_t<std::is_same_v<decltype(std::declval<T>().position.x), double> &&
                        std::is_same_v<decltype(std::declval<T>().position.y), double>>>
    : public std::true_type {};

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
 * @param path Sequence of points to fit the spline to.
 * @param precision Number of interpolated points between each pair of original points.
 * @param order Order of the B-spline.
 * @param coeffs_ratio Ratio to determine the number of coefficients for the spline.
 * @return std::vector<T> Sequence of points representing the fitted spline.
 *
 * @note This function requires the GNU Scientific Library (GSL) for spline fitting.
 */
template <typename T>
std::vector<T> fit_spline(const std::vector<T> &path, int precision, int order,
                          float coeffs_ratio) {
  static_assert(HasDefaultConstructor<T>::value, "T must be default constructible");
  static_assert(IsHashable<T>::value, "T must be hashable");
  static_assert(HasEqualityOperator<T>::value, "T must have operator==");
  static_assert(HasPosition<T>::value, "T must have a position member");
  static_assert(HasPositionXY<T>::value, "T.position must have x and y members");
  static_assert(IsCopyConstructor<T>::value, "T must be copyable");
  static_assert(HasEuclideanDistance<T>::value, "T.position must have a euclidean_distance method");
  static_assert(PositionXYAreDouble<T>::value, "T.position.x and T.position.y must be double");

  size_t n = path.size();
  if (n < 2) {
    return path;
  }

  // Garantee ncoeffs >= order -> mathematical requirement for B-splines
  size_t ncoeffs = static_cast<size_t>(static_cast<float>(n) / coeffs_ratio);
  if (ncoeffs < static_cast<size_t>(order)) {
    return path;
  }

  if (ncoeffs > n) {
    ncoeffs = n;
  }

  // Number of breakpoints in the B-spline (knots)
  const int nbreak = static_cast<int>(ncoeffs) - order + 2;

  if (nbreak < 2 || n == 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Too few points to calculate spline while executing 'fit_spline'"
                 "Number of cones was %i",
                 static_cast<int>(n));
    return path;
  }
  // -------- INITIALIZE GSL WORKSPACES --------
  gsl_bspline_workspace *bw =
      ::gsl_bspline_alloc(static_cast<size_t>(order), static_cast<size_t>(nbreak));
  gsl_bspline_workspace *cw = ::gsl_bspline_alloc(order, nbreak);

  gsl_vector *B = ::gsl_vector_alloc(ncoeffs);
  gsl_vector *C = ::gsl_vector_alloc(ncoeffs);
  gsl_vector *c = ::gsl_vector_alloc(ncoeffs);
  gsl_vector *c2 = ::gsl_vector_alloc(ncoeffs);

  gsl_vector *w = ::gsl_vector_alloc(n);
  gsl_vector *x_values = ::gsl_vector_alloc(n);
  gsl_vector *y_values = ::gsl_vector_alloc(n);
  gsl_vector *i_values = ::gsl_vector_alloc(n);

  gsl_matrix *X = ::gsl_matrix_alloc(n, ncoeffs);
  gsl_matrix *Y = ::gsl_matrix_alloc(n, ncoeffs);

  gsl_matrix *cov = ::gsl_matrix_alloc(ncoeffs, ncoeffs);
  gsl_matrix *cov2 = ::gsl_matrix_alloc(ncoeffs, ncoeffs);

  gsl_multifit_linear_workspace *mw = ::gsl_multifit_linear_alloc(n, ncoeffs);
  gsl_multifit_linear_workspace *mw2 = ::gsl_multifit_linear_alloc(n, ncoeffs);

  double chisq = 0.0;
  double chisq2 = 0.0;

  // -------- FILL INPUT DATA AND WEIGHTS --------
  for (size_t i = 0; i < n; i++) {
    ::gsl_vector_set(i_values, i, static_cast<double>(i));
    ::gsl_vector_set(x_values, i, path[i].position.x);
    ::gsl_vector_set(y_values, i, path[i].position.y);

    // Weight points based on local spacing (closer points get larger weight)
    double dist_next = 1.0;
    if (i + 1 < n) {
      dist_next = path[i].position.euclidean_distance(path[i + 1].position);
    } else if (i > 0) {
      dist_next = path[i].position.euclidean_distance(path[i - 1].position);
    }

    if (dist_next < 1e-6) {
      dist_next = 1e-6;
    }

    ::gsl_vector_set(w, i, 1.0 / (dist_next * dist_next));
  }

  // -------- SET UNIFORM KNOTS FOR BOTH SPLINES --------
  double t_min = 0.0;
  double t_max = static_cast<double>(n - 1);
  (void)::gsl_bspline_knots_uniform(t_min, t_max, bw);
  (void)::gsl_bspline_knots_uniform(t_min, t_max, cw);

  // -------- BUILD DESIGN MATRICES --------
  // Evaluate each basis function at each input index
  for (int i = 0; i < static_cast<int>(n); i++) {
    (void)::gsl_bspline_eval(i, B, bw);
    (void)::gsl_bspline_eval(i, C, cw);

    // Fill design matrices for x and y least squares
    for (int j = 0; j < static_cast<int>(ncoeffs); j++) {
      ::gsl_matrix_set(X, i, j, ::gsl_vector_get(B, j));
      ::gsl_matrix_set(Y, i, j, ::gsl_vector_get(C, j));
    }
  }

  // -------- LEAST SQUARES FIT FOR X AND Y --------
  (void)::gsl_multifit_wlinear(X, w, x_values, c, cov, &chisq, mw);
  (void)::gsl_multifit_wlinear(Y, w, y_values, c2, cov2, &chisq2, mw2);

  // -------- EVALUATE THE SPLINE  --------
  std::vector<T> path_eval;
  path_eval.reserve(n * precision);
  for (int i = 0; i < static_cast<int>(n); i++) {
    for (int j = 0; j < precision; j++) {
      // Compute parameter t inside segment i
      double t = static_cast<double>(i) + static_cast<double>(j) / precision;
      if (t > t_max) {
        t = t_max;
      }

      (void)::gsl_bspline_eval(t, B, bw);
      (void)::gsl_bspline_eval(t, C, cw);

      // Compute x and y via estimated spline coefficients
      double xi = 0.0;
      double yi = 0.0;
      double err = 0.0;
      (void)::gsl_multifit_linear_est(B, c, cov, &xi, &err);
      (void)::gsl_multifit_linear_est(C, c2, cov2, &yi, &err);

      // Create new spline point
      T new_element;
      new_element.position.x = xi;
      new_element.position.y = yi;
      path_eval.push_back(new_element);
    }
  }

  // -------- CLEANUP GSL RESOURCES --------
  ::gsl_bspline_free(bw);
  ::gsl_bspline_free(cw);
  ::gsl_vector_free(B);
  ::gsl_vector_free(C);
  ::gsl_vector_free(c);
  ::gsl_vector_free(c2);
  ::gsl_vector_free(w);
  ::gsl_vector_free(x_values);
  ::gsl_vector_free(y_values);
  ::gsl_vector_free(i_values);
  ::gsl_matrix_free(X);
  ::gsl_matrix_free(Y);
  ::gsl_matrix_free(cov);
  ::gsl_matrix_free(cov2);
  ::gsl_multifit_linear_free(mw);
  ::gsl_multifit_linear_free(mw2);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END fitSpline with %i points",
               static_cast<int>(path_eval.size()));

  return path_eval;
}
/**
 * @brief This function takes three sequences of points (center, left, right), fits splines to them
 * using GSL interpolation methods, and returns three sequences of points representing the fitted
 * splines. All three splines share the same parametric domain based on the center line's arc
 * length.
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
 * @param center Sequence of points representing the center line.
 * @param left Sequence of points representing the left boundary.
 * @param right Sequence of points representing the right boundary.
 * @param precision Number of interpolated points between each pair of original points.
 * @param order Order of the interpolation: 2=linear, 3=cubic spline, other=Akima spline.
 * @return TripleSpline<T> Structure containing three spline sequences: center, left, and right.
 *
 * @note This function requires the GNU Scientific Library (GSL) for spline interpolation.
 * @note All three input sequences must have the same size and at least 2 points.
 */
template <typename T>
TripleSpline<T> fit_triple_spline(const std::vector<T> &center, const std::vector<T> &left,
                                  const std::vector<T> &right, int precision, int order) {
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

  // Check if we have enough points and all sequences have the same size
  if (n < 2 || left.size() != n || right.size() != n) {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Invalid input for fit_triple_spline: center has %zu points, "
                 "left has %zu points, right has %zu points. All must be >= 2 and equal.",
                 center.size(), left.size(), right.size());
    return result;
  }

  // -------- COMPUTE PARAMETRIC DOMAIN BASED ON ARC LENGTH --------
  // Use cumulative arc length of the center line as the parameter t
  std::vector<double> t_values(n);
  t_values[0] = 0.0;

  for (size_t i = 1; i < n; ++i) {
    double dist = center[i].position.euclidean_distance(center[i - 1].position);
    t_values[i] = t_values[i - 1] + dist;
  }

  // -------- EXTRACT COORDINATE DATA FROM INPUT POINTS --------
  std::vector<double> center_x(n);
  std::vector<double> center_y(n);
  std::vector<double> left_x(n);
  std::vector<double> left_y(n);
  std::vector<double> right_x(n);
  std::vector<double> right_y(n);

  for (size_t i = 0; i < n; ++i) {
    center_x[i] = center[i].position.x;
    center_y[i] = center[i].position.y;
    left_x[i] = left[i].position.x;
    left_y[i] = left[i].position.y;
    right_x[i] = right[i].position.x;
    right_y[i] = right[i].position.y;
  }

  // -------- INITIALIZE GSL INTERPOLATION --------
  gsl_interp_accel *acc = ::gsl_interp_accel_alloc();

  // Select interpolation type based on order parameter
  const gsl_interp_type *interp_type = gsl_interp_linear;
  if (order == 3) {
    interp_type = gsl_interp_cspline;
  }

  // -------- ALLOCATE SPLINES FOR ALL COORDINATES --------
  // We need 6 splines total: (center, left, right) × (x, y)
  gsl_spline *spline_center_x = ::gsl_spline_alloc(interp_type, n);
  gsl_spline *spline_center_y = ::gsl_spline_alloc(interp_type, n);
  gsl_spline *spline_left_x = ::gsl_spline_alloc(interp_type, n);
  gsl_spline *spline_left_y = ::gsl_spline_alloc(interp_type, n);
  gsl_spline *spline_right_x = ::gsl_spline_alloc(interp_type, n);
  gsl_spline *spline_right_y = ::gsl_spline_alloc(interp_type, n);

  // -------- INITIALIZE SPLINES WITH DATA --------
  // Each spline interpolates one coordinate as a function of parameter t
  (void)::gsl_spline_init(spline_center_x, t_values.data(), center_x.data(), n);
  (void)::gsl_spline_init(spline_center_y, t_values.data(), center_y.data(), n);
  (void)::gsl_spline_init(spline_left_x, t_values.data(), left_x.data(), n);
  (void)::gsl_spline_init(spline_left_y, t_values.data(), left_y.data(), n);
  (void)::gsl_spline_init(spline_right_x, t_values.data(), right_x.data(), n);
  (void)::gsl_spline_init(spline_right_y, t_values.data(), right_y.data(), n);

  // -------- EVALUATE SPLINES AT HIGHER RESOLUTION --------
  const double t_min = t_values[0];
  const double t_max = t_values[n - 1];
  const size_t total_points = n * precision;

  result.center.reserve(total_points);
  result.left.reserve(total_points);
  result.right.reserve(total_points);

  for (size_t i = 0; i < n; ++i) {
    for (int j = 0; j < precision; ++j) {
      // Compute parameter t for this evaluation point within segment i
      double t_start = t_values[i];
      double t_end = (i + 1 < n) ? t_values[i + 1] : t_max;
      double t = t_start + (t_end - t_start) * (static_cast<double>(j) / precision);

      // Clamp t to valid range
      if (t > t_max) {
        t = t_max;
      }
      if (t < t_min) {
        t = t_min;
      }

      // -------- EVALUATE ALL SPLINES AT PARAMETER t --------
      T point_center;
      T point_left;
      T point_right;

      point_center.position.x = ::gsl_spline_eval(spline_center_x, t, acc);
      point_center.position.y = ::gsl_spline_eval(spline_center_y, t, acc);

      point_left.position.x = ::gsl_spline_eval(spline_left_x, t, acc);
      point_left.position.y = ::gsl_spline_eval(spline_left_y, t, acc);

      point_right.position.x = ::gsl_spline_eval(spline_right_x, t, acc);
      point_right.position.y = ::gsl_spline_eval(spline_right_y, t, acc);

      result.center.push_back(point_center);
      result.left.push_back(point_left);
      result.right.push_back(point_right);
    }
  }

  // -------- CLEANUP GSL RESOURCES --------
  ::gsl_spline_free(spline_center_x);
  ::gsl_spline_free(spline_center_y);
  ::gsl_spline_free(spline_left_x);
  ::gsl_spline_free(spline_left_y);
  ::gsl_spline_free(spline_right_x);
  ::gsl_spline_free(spline_right_y);
  ::gsl_interp_accel_free(acc);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "END fit_triple_spline with %zu center, %zu left, %zu right points",
               result.center.size(), result.left.size(), result.right.size());

  return result;
}

#endif  // SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_