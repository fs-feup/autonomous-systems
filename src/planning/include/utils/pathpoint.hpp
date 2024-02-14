#ifndef SRC_PLANNING_PLANNING_INCLUDE_UTILS_PATHPOINT_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_UTILS_PATHPOINT_HPP_

#include <cmath>
#include <fstream>
#include <vector>

/**
 * PathPoint class. Stores the x, y and v values of the car
 */
class PathPoint {
  float x, y, v;

 public:
  PathPoint(float x, float y, float v = 0);

  float getX() const;

  float getY() const;

//float getV() const;

  /**
   * Euclidean distance to another path point
   */
  float getDistanceTo(PathPoint *dest);

  bool operator==(const PathPoint &p) const;
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_UTILS_PATHPOINT_HPP_
