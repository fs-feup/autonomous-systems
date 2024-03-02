#ifndef SRC_PLANNING_INCLUDE_PLANNING_GLOBAL_PATH_PLANNER_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_GLOBAL_PATH_PLANNER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "../utils/pathpoint.hpp"
#include "./track.hpp"

/**
 * PathPlanner class. Contains the best path calculation methods and stores the
 * results and inputs
 */

class GlobalPathPlanner {
  Track *track;                        // track input data
  std::vector<PathPoint *> finalPath;  // path calculation result

 public:
  /**
   * Constructor
   * @param track pathplanner track input data
   */
  explicit GlobalPathPlanner(Track *track);

  /**
   * Writes the output/result path to a file
   */
  void writeFinalPath(const std::string &filePrefix);

  // void rrt();
  // void astar();
  // other future algorithms

  /**
   * Retrieves the final path using copied values. Used for testing purposes
   * @return
   */
  std::vector<std::pair<float, float>> getPath();

  // void sendPath(); // TODO(andre): send path to controller
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_GLOBAL_PATH_PLANNER_HPP_
