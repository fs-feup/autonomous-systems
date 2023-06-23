#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <utility>
#include <vector>
#include <map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "./track.hpp"
#include "../utils/position.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> DT;
typedef K::Point_2 Point;

/**
 * PathPlanner class. Contains the best path calculation methods and stores the
 * results and inputs
 */

class LocalPathPlanner {
  Track track; /**<track input data */
  float orientation = 0;

 public:
  /**
   * Constructor
   * @param track pathplanner track input data
   */
  LocalPathPlanner();

  void setOrientation(float theta);

  bool vectorDirection(Position* p1, Position* p2);

  std::vector<Position*> processNewArray(Track* cone_array);

  float euclideanDist(Position* p1, Position* p2);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
