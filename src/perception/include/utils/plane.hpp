#ifndef PLANE_HPP
#define PLANE_HPP

#include <string>
#include <pcl/point_types.h>

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
     * @brief Getter method to retrieve the 'a' coefficient of the plane equation.
     * 
     * @return The 'a' coefficient of the plane equation.
     */
    double getA();

    /**
     * @brief Getter method to retrieve the 'b' coefficient of the plane equation.
     * 
     * @return The 'b' coefficient of the plane equation.
     */
    double getB();

    /**
     * @brief Getter method to retrieve the 'c' coefficient of the plane equation.
     * 
     * @return The 'c' coefficient of the plane equation.
     */
    double getC();

    /**
     * @brief Getter method to retrieve the 'd' coefficient of the plane equation.
     * 
     * @return The 'd' coefficient of the plane equation.
     */
    double getD();

    /**
     * @brief Setter method to set the 'a' coefficient of the plane equation.
     * 
     * @param newA The new value for the 'a' coefficient.
     */
    void setA(double newA);

    /**
     * @brief Setter method to set the 'b' coefficient of the plane equation.
     * 
     * @param newB The new value for the 'b' coefficient.
     */
    void setB(double newB);

    /**
     * @brief Setter method to set the 'c' coefficient of the plane equation.
     * 
     * @param newC The new value for the 'c' coefficient.
     */
    void setC(double newC);

    /**
     * @brief Setter method to set the 'd' coefficient of the plane equation.
     * 
     * @param newD The new value for the 'd' coefficient.
     */
    void setD(double newD);

    /**
     * @brief Calculates the distance from a point to the plane.
     * 
     * @param point The point for which the distance to the plane is calculated.
     * @return The distance from the point to the plane.
     */
    double getDistanceToPoint(pcl::PointXYZI point);
};

#endif  // PLANE_HPP
