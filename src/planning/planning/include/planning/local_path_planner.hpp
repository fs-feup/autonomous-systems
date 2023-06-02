#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "./track.hpp"
#include "./position.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> DT;
typedef K::Point_2 Point;


using namespace std;

/**
 * PathPlanner class. Contains the best path calculation methods and stores the
 * results and inputs
 */

class LocalPathPlanner {
  Track track; /**<track input data */

 public:
  /**
   * Constructor
   * @param track pathplanner track input data
   */
  LocalPathPlanner();

  vector<Position*> process_new_array(Track* cone_array);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_LOCAL_PATH_PLANNER_HPP_
