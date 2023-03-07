#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATHPLANNER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATHPLANNER_HPP_

#include <utility>
#include <vector>

#include "track.hpp"

using namespace std;

/**
 * PathPlanner class. Contains the best path calculation methods and stores the
 * results and inputs
 */

class PathPlanner {
  Track* track;                /**<track input data */
  vector<Position*> finalPath; /**<path calculation result */

 public:
  /**
   * Constructor
   * @param track pathplanner track input data
   */
  explicit PathPlanner(Track* track);

  /**
   * Writes the output/result path to a file
   */
  void writeFinalPath();

  /**
   * Calculates the track central path
   */
  void middlePath();  // TODO(andre): calculate middle path using track info

  // void rrt();
  // void astar();
  // other future algorithms

  /**
   * Retrieves the final path using copied values. Used for testing purposes
   * @return
   */
  vector<pair<float, float>> getPath();

  // void sendPath(); // TODO(andre): send path to controller
};

#endif // RC_PLANNING_PLANNING_INCLUDE_PLANNING_PATHPLANNER_HPP_
