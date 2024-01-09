#ifndef SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSITION_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSITION_HPP_

#include <cmath>
#include <fstream>
#include <vector>

/**
 * Position class. Stores the x and y values of the car
 */
class Position {
  float x, y;

 public:
  Position(float x, float y);

  float getX() const;

  float getY() const;

  /**
   * Euclidean distance to another position
   */
  float getDistanceTo(Position *dest);

  bool operator==(const Position &p) const;
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSITION_HPP_
