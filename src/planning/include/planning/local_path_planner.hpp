#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include "../utils/pathpoint.hpp"
#include "./track.hpp"
#include "rclcpp/rclcpp.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> DT;
typedef K::Point_2 Point;

/**
 * @brief LocalPathPlanner class for generating local paths.
 *
 * The LocalPathPlanner class contains methods for calculating the best local
 * path and stores input data and results related to path planning.
 */
class LocalPathPlanner {
  Track track;  // track input data

 public:
  /**
   * @brief Constructor for LocalPathPlanner.
   *
   * @param track Pointer to the path planner track input data.
   */
  LocalPathPlanner();

  /**
   * @brief Determine if two positions align in a certain direction.
   *
   * This function checks whether the direction vector formed by two positions
   * aligns with a specified direction (prev_vx, prev_vy).
   *
   * @param p1 Pointer to the first position.
   * @param p2 Pointer to the second position.
   * @param prev_vx Previous direction vector's x-component.
   * @param prev_vy Previous direction vector's y-component.
   * @return True if the direction aligns; false otherwise.
   */
  bool vector_direction(PathPoint *p1, PathPoint *p2, float prev_vx, float prev_vy);

  /**
   * @brief Process an array of cones to generate a local path.
   *
   * This function processes an array of cones representing a track and
   * generates a local path by selecting positions based on certain criteria.
   *
   * @param cone_array Pointer to the array of cones representing the track.
   * @return A vector of pointers to Position objects representing the generated
   * path.
   * @details The function utilizes Delaunay triangulation (CGAL) and
   * direction-based selection of positions to create a meaningful local path.
   */
  std::vector<PathPoint *> processNewArray(Track *cone_array);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
