#ifndef SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_
#define SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_

#include <gsl/gsl_bspline.h>
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
struct has_default_constructor : std::false_type {};

template <typename T>
struct has_default_constructor<T, std::void_t<decltype(T())>> : std::true_type {};

// Check for hash function
template <typename T, typename = void>
struct is_hashable : std::false_type {};

template <typename T>
struct is_hashable<T, std::void_t<decltype(std::hash<T>{}(std::declval<T>()))>> : std::true_type {};

// Check for operator==
template <typename T, typename = void>
struct has_equal_operator : std::false_type {};

template <typename T>
struct has_equal_operator<T, std::void_t<decltype(std::declval<T>() == std::declval<T>())>>
    : std::true_type {};

// Check for position member
template <typename T, typename = void>
struct has_position : std::false_type {};

template <typename T>
struct has_position<T, std::void_t<decltype(std::declval<T>().position)>> : std::true_type {};

// Check for position.x and position.y members
template <typename T, typename = void>
struct has_position_x_y : std::false_type {};

template <typename T>
struct has_position_x_y<
    T, std::void_t<decltype(std::declval<T>().position.x), decltype(std::declval<T>().position.y)>>
    : std::true_type {};

// Check for copy constructor
template <typename T, typename = void>
struct is_copy_constructible : std::false_type {};

template <typename T>
struct is_copy_constructible<T, std::void_t<decltype(T(std::declval<T>()))>> : std::true_type {};

// Check for euclidean_distance method
template <typename T, typename = void>
struct has_euclidean_distance : std::false_type {};

template <typename T>
struct has_euclidean_distance<T, std::void_t<decltype(std::declval<T>().position.euclidean_distance(
                                     std::declval<T>().position))>> : std::true_type {};

// Check for position.x and position.y being double
template <typename T, typename = void>
struct position_x_y_are_double : std::false_type {};

template <typename T>
struct position_x_y_are_double<
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
 * @param cone_seq Sequence of points to fit the spline to. This sequence will have duplicates
 * removed.
 * @return std::vector<T> Sequence of points representing the fitted spline.
 *
 * @note This function requires the GNU Scientific Library (GSL) for spline fitting.
 */
template <typename T>
std::vector<T> fit_spline(int precision, int order, float coeffs_ratio, std::vector<T> cone_seq) {
  static_assert(has_default_constructor<T>::value, "T must be default constructible");
  static_assert(is_hashable<T>::value, "T must be hashable");
  static_assert(has_equal_operator<T>::value, "T must have operator==");
  static_assert(has_position<T>::value, "T must have a position member");
  static_assert(has_position_x_y<T>::value, "T.position must have x and y members");
  static_assert(is_copy_constructible<T>::value, "T must be copyable");
  static_assert(has_euclidean_distance<T>::value,
                "T.position must have a euclidean_distance method");
  static_assert(position_x_y_are_double<T>::value, "T.position.x and T.position.y must be double");

  std::unordered_set<T> seen;

  // Use std::remove_if and a lambda function to remove duplicates
  auto iterator = std::remove_if(cone_seq.begin(), cone_seq.end(), [&seen](const auto &ptr) {
    return !seen.insert(ptr).second;  // insert returns a pair, where .second is false if the
                                      // element already existed
  });

  // Finally, erase their allocated space
  cone_seq.erase(iterator, cone_seq.end());

  size_t n = cone_seq.size();
  auto ncoeffs = static_cast<size_t>(static_cast<float>(n) / coeffs_ratio);  // n > = ncoeffs
  const int nbreak = static_cast<int>(ncoeffs) - order + 2;

  if (nbreak < 2 || n == 0) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "Too few points to calculate spline while executing 'fit_spline'"
                "Number of cones was %i",
                static_cast<int>(n));
    return cone_seq;
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
  bw = gsl_bspline_alloc(static_cast<size_t>(order), static_cast<size_t>(nbreak));
  cw = gsl_bspline_alloc(static_cast<size_t>(order), static_cast<size_t>(nbreak));
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
  for (size_t i = 0; i < n; i++) {
    gsl_vector_set(i_values, i, static_cast<double>(i));
    gsl_vector_set(x_values, i, cone_seq[i].position.x);
    gsl_vector_set(y_values, i, cone_seq[i].position.y);
    // closer cones more important(better stability)
    gsl_vector_set(
        w, i,
        1.0 / pow(cone_seq[i].position.euclidean_distance(cone_seq[(i + 1) % n].position), 2));
  }

  // Set i range within cone set length
  gsl_bspline_knots_uniform(0, n, bw);
  gsl_bspline_knots_uniform(0, n, cw);

  /* construct the fit matrix X */
  for (int i = 0; i < static_cast<int>(n); i++) {
    /* compute B_j(xi) for all j */
    gsl_bspline_eval(i, B, bw);
    gsl_bspline_eval(i, C, cw);

    /* fill in row i of X */
    for (int j = 0; j < static_cast<int>(ncoeffs); j++) {
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
  double xi;
  double yi;
  double yerr;
  double yerr2;
  std::vector<double> i_eval;
  std::vector<double> x_eval;
  std::vector<double> y_eval;
  std::vector<T> cone_seq_eval;

  // Calculate the desired amount of spline points and add them to
  // "cone_seq_eval"
  for (int i = 0; i < static_cast<int>(n); i++) {
    for (int j = 0; j < precision; j++) {  // iterate over decimal numbers
      gsl_bspline_eval(
          static_cast<float>(i) + static_cast<float>(j) / static_cast<float>(precision), B, bw);
      gsl_bspline_eval(
          static_cast<float>(i) + static_cast<float>(j) / static_cast<float>(precision), C, cw);
      gsl_multifit_linear_est(B, c, cov, &xi, &yerr);
      gsl_multifit_linear_est(C, c2, cov2, &yi, &yerr2);
      i_eval.push_back(i);
      x_eval.push_back(xi);
      y_eval.push_back(yi);
      T new_element;
      new_element.position.x = xi;
      new_element.position.y = yi;
      cone_seq_eval.push_back(new_element);
      // if (j == 0 && i == static_cast<int>(n) - 1) {
      //   break;  // Decimals can't go over last int
      // }
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
  gsl_vector_free(c2);
  gsl_vector_free(w);
  gsl_matrix_free(cov);
  gsl_matrix_free(cov2);
  gsl_multifit_linear_free(mw);
  gsl_multifit_linear_free(mw2);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "END fitSpline with %i points",
               static_cast<int>(cone_seq_eval.size()));

  // To access spline points uncomment these lines
  // std::cout << "track spline : " << std::endl;
  // for (int i = 0; i < static_cast<int>(cone_seq_eval.size()); i++) {
  //   std::cout << "(" << cone_seq_eval[i]->getX() << "," << cone_seq_eval[i]->getY() << "),";
  // }
  // std::cout << std::endl;

  return cone_seq_eval;
}

#endif  // SRC_PLANNING_INCLUDE_UTILS_SPLINES_HPP_