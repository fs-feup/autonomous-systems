#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include "../utils/position.hpp"
#include "./track.hpp"
#include "rclcpp/rclcpp.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> DT;
typedef K::Point_2 Point;

/**
 * PathPlanner class. Contains the best path calculation methods and stores the
 * results and inputs
 */

class LocalPathPlanner {
  Track track;  // track input data
  float orientation = 0;

 public:
  /**
   * Constructor
   * @param track pathplanner track input data
   */
  LocalPathPlanner();

  void setOrientation(float theta);

  bool vector_direction(Position* p1, Position* p2, float prev_vx, float prev_vy);

  std::vector<Position*> processNewArray(Track* cone_array);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
