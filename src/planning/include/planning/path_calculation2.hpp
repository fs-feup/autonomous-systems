#ifndef SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "rclcpp/rclcpp.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> DT;
typedef K::Point_2 Point;

using Cone = common_lib::structures::Cone;
using PathPoint = common_lib::structures::PathPoint;

/**
 * @brief LocalPathPlanner class for generating local paths.
 *
 * The LocalPathPlanner class contains methods for calculating the best local
 * path and stores input data and results related to path planning.
 */
class PathCalculation {
  double dist_threshold = 7.0;

 public:
  /**
   * @brief Constructor for LocalPathPlanner.
   *
   * @param dist_threshold Distance value where triangulations are discarded.
   */
  PathCalculation(double dist_threshold);

  /**
   * @brief Process an array of cones to generate a local path.
   *
   * This function processes an array of cones representing a track and
   * generates a local path by selecting positions based on certain criteria.
   *
   * @param cone_array Pointer to the array of cones representing the track.
   * @return A vector of pointers to PathPoint objects representing the generated
   * path.
   * @details The function utilizes Delaunay triangulation (CGAL) and
   * direction-based selection of positions to create a meaningful local path.
   */
  std::vector<PathPoint> processDelaunayTriangulations(std::vector<Cone> cones);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_
