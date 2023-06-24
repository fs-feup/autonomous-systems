#ifndef SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSITION_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSITION_HPP_

#include <cmath>
#include <vector>
#include <fstream>

/**
 * Position class. Stores the x and y values of the car
 */
class Position {
  float x, y;

 public:
  Position(float x, float y);

  float getX() const;

  // void setX(float x);

  float getY() const;

  // void setY(float y);

  /**
   * Euclidean distance to another position
   */
  float getDistanceTo(Position* dest);

  bool operator==(const Position& p) const;

  //std::ostream& operator<<(std::ostream& os,const Position& p);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_UTILS_POSITION_HPP_
