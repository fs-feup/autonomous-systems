#include <utils/plane.hpp>

Plane::Plane(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}

Plane::Plane() : a(0), b(0), c(0), d(0) {}

double Plane::getDistanceToPoint(pcl::PointXYZI point) {
  double numerator = std::abs(a * point.x + b * point.y + c * point.z + d);
  double denominator = std::sqrt(a * a + b * b + c * c);
  return numerator / denominator;
}