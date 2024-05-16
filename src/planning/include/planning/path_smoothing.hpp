#ifndef SRC_PLANNING_INCLUDE_PLANNING_PATH_SMOOTHING_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PATH_SMOOTHING_HPP_

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "planning/track.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/cone.hpp"
#include "utils/files.hpp"
#include "utils/pathpoint.hpp"

class PathSmoothing {
  std::vector<PathPoint *> path;

  int spline_order_ = 3;
  float spline_coeffs_ratio_ = 3;
  int spline_precision_ = 10;

 public:
  /**
   * @brief Construct a new default Path Smoothing object
   *
   */
  PathSmoothing() = default;

  /**
   * @brief Construct a new Path Smoothing object
   *
   */
  PathSmoothing(int spline_order, float spline_coeffs_ratio, int spline_precision);

  /**
   * @brief Get the Path object
   *
   * @return path (vector of pointers to PathPoints)
   */
  std::vector<PathPoint *> getPath();

  /**
   * @brief adds point to path
   *
   * @param xValue x coordinate of the path point
   * @param yValue y coordinate of the path point
   * @param vValue value for velocity
   */
  void addPathPoint(float xValue, float yValue, float vValue);

  /**
   * @brief Get the number of points in the path
   *
   * @return number of points in the path
   */
  int getPointAmount();

  /**
   * @brief logs the cones in path in the following format: (x,y),...,(xn,yn)
   *
   */
  void logPathPoints();

  /**
   * @brief Fill the track data from a file.
   *
   * @param path The path to the file containing track data.
   */
  void fillPath(const std::string &path);

  /**
   * @brief Check if the direction of two cones is consistent.
   *
   * @param c1 The first cone.
   * @param c2 The second cone.
   * @param prev_vx The previous x-direction.
   * @param prev_vy The previous y-direction.
   * @return True if the direction is consistent, false otherwise.
   */
  bool vector_direction(PathPoint *c1, PathPoint *c2, float prev_vx, float prev_vy);

  /**
   * @brief Orders the path to fit spline
   *
   * @param unord_path unordered path points
   * @return ordered path points
   */
  std::vector<PathPoint *> orderPath(std::vector<PathPoint *> *unord_path);

  /**
   * @brief calls pathSmoother with default values for precision,
   *  order and coeffs_ratio that were determined to work well
   *
   * @param a_path path to be smoothed
   */
  void defaultSmoother(std::vector<PathPoint *> &a_path);

  /**
   * @brief Fits spline to path
   *
   * @param precision Number of spline points for each input PathPoint
   * @param order Order of the spline
   * @param coeffs_ratio Ratio of coefficients to PathPoints
   * @param path Ordered PathPoints
   * @return Smoothed path
   */
  std::vector<PathPoint *> splineSmoother(int precision, int order, float coeffs_ratio,
                                          std::vector<PathPoint *> input_path);

  /**
   * @brief Function designed to smooth the path via spline
   *
   * @param precision Number of spline points for each input PathPoint
   * @param order Order of the spline
   * @param coeffs_ratio Ratio of coefficients to PathPoints
   * @param unord_path Ordered PathPoints
   * @return Smoothed path
   */
  std::vector<PathPoint *> pathSmoother(int precision, int order, float coeffs_ratio,
                                        std::vector<PathPoint *> unord_path);
};
#endif