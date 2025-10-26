#include <utils/plane.hpp>

Plane::Plane(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}

Plane::Plane() : a(0), b(0), c(0), d(0) {}

double Plane::get_a() const { return a; }

double Plane::get_b() const { return b; }

double Plane::get_c() const { return c; }

double Plane::get_d() const { return d; }

double Plane::get_distance_to_point(PointXYZIR point) const {
  double numerator = std::abs(a * point.x + b * point.y + c * point.z + d);
  double denominator = std::sqrt(a * a + b * b + c * c);
  return numerator / denominator;
}

Plane& Plane::operator+=(const Plane& other) {
  a += other.a;
  b += other.b;
  c += other.c;
  d += other.d;
  return *this;
}

Plane& Plane::operator/=(double scalar) {
  if (scalar != 0.0) {
    a /= scalar;
    b /= scalar;
    c /= scalar;
    d /= scalar;
  }
  return *this;
}