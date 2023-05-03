#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_SLAMPATHPLANNER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_SLAMPATHPLANNER_HPP_

#include <utility>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
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

class SlamPathPlanner {
  Track* track;                /**<track input data */
  vector<Position*> path; /**<path calculation result */

 public:
  /**
   * Constructor
   * @param track pathplanner track input data
   */
  explicit SlamPathPlanner();

  void processNewArray(Track* cone_array);

};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_SLAMPATHPLANNER_HPP_
