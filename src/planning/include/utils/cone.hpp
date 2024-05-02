#ifndef SRC_PLANNING_PLANNING_INCLUDE_UTILS_CONE_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_UTILS_CONE_HPP_

#include <cmath>
#include <string>
#include <vector>

/**
 * Cone class. Stores the id, x and y values of a cone
 */
class Cone {
  int id;
  float x;
  float y;

 public:
  Cone(int id, float x, float y);

  int getId() const;

  float getX() const;

  void setX(float x);

  float getY() const;

  void setY(float y);

  std::string print();

  /**
   * Euclidean distance to another position
   */
  double getDistanceTo(const Cone *dest) const;

  double squaredDistanceTo(const Cone *dest) const;
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_UTILS_CONE_HPP_
