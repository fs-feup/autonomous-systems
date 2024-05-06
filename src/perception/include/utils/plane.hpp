#ifndef PLANE_HPP
#define PLANE_HPP

#include <pcl/point_types.h>

#include <string>

/**
 * @brief The Plane class represents a 3D plane defined by its equation ax + by + cz + d = 0.
 *
 * This class provides methods to manipulate and retrieve parameters of a plane,
 * as well as calculate the distance from a point to the plane.
 */
class Plane {
 private:
  double a;  // Coefficient 'a' of the plane equation
  double b;  // Coefficient 'b' of the plane equation
  double c;  // Coefficient 'c' of the plane equation
  double d;  // Coefficient 'd' of the plane equation

 public:
  /**
   * @brief Constructs a new Plane object with the specified coefficients.
   *
   * @param a Coefficient 'a' of the plane equation.
   * @param b Coefficient 'b' of the plane equation.
   * @param c Coefficient 'c' of the plane equation.
   * @param d Coefficient 'd' of the plane equation.
   */
  Plane(double a, double b, double c, double d);

  /**
   * @brief Constructs a new Plane object with default coefficients (0).
   */
  Plane();

  /**
   * @brief Calculates the distance from a point to the plane.
   *
   * @param point The point for which the distance to the plane is calculated.
   * @return The distance from the point to the plane.
   */
  double getDistanceToPoint(pcl::PointXYZI point);

  /**
   * @brief Overloads the += operator to add another plane to this plane.
   *
   * @param other The plane to be added.
   * @return Reference to the modified plane.
   */
  Plane& operator+=(const Plane& other);

  /**
   * @brief Overloads the /= operator to divide a plane by a scalar.
   *
   * @param scalar The scalar to divide the plane.
   * @return Reference to the modified plane.
   */
  Plane& operator/=(double scalar);
};

#endif  // PLANE_HPP
