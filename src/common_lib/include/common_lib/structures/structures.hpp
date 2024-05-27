#ifndef STRUCTURES_HPP_
#define STRUCTURES_HPP_
#include <cmath>

class Point {
 public:
  double x_;
  double y_;

  // Default constructor
  Point() : x_(0.0), y_(0.0) {}

  // Constructor
  Point(double x, double y) : x_(x), y_(y) {}

  Point operator+(const Point& p) const { return Point(x_ + p.x_, y_ + p.y_); }

  Point operator-(const Point& p) const { return Point(x_ - p.x_, y_ - p.y_); }

  double euclidean_distance(const Point& p) const {
    return std::sqrt(pow(x_ - p.x_, 2) + pow(y_ - p.y_, 2));
  }
};

class Pose {
 public:
  Point cg_;
  Point rear_axis_;
  double heading_;
  double orientation_;
  double velocity_;

  // Default constructor
  Pose() : heading_(0.0), orientation_(0.0), velocity_(0.0) {}

  // Constructor
  Pose(Point cg, Point rear_axis, double heading, double orientation, double velocity)
      : cg_(cg),
        rear_axis_(rear_axis),
        heading_(heading),
        orientation_(orientation),
        velocity_(velocity){};
};
#endif  // STRUCTURES_HPP_