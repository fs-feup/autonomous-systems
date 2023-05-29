#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_CONE_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_CONE_HPP_

#include <cmath>
#include <vector>
#include <string>
/**
 * Position class. Stores the x and y values of the car
 */
class Cone {
    int id;
    float x, y;

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
  float getDistanceTo(Cone* dest);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_CONE_HPP_
