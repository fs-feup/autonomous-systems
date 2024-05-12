#include <cmath>

class Point {
 public:
  double x;
  double y;

  // Default constructor
  Point() {
    x = 0.0;
    y = 0.0;
  }

  // Constructor
  Point(double x, double y) : x(x), y(y) {}

  Point operator+(const Point& p) const { return Point(x + p.x, y + p.y); }

  Point operator-(const Point& p) const { return Point(x - p.x, y - p.y); }

  double euclidean_distance(const Point& p) const {
    return std::sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
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
  Pose() {
    cg_ = Point();
    rear_axis_ = Point();
    heading_ = 0.0;
    orientation_ = 0.0;
    velocity_ = 0.0;
  }

  // Constructor
  Pose(Point cg, Point rear_axis, double heading, double orientation, double velocity) {
    this->cg_ = cg;
    this->rear_axis_ = rear_axis;
    this->heading_ = heading;
    this->orientation_ = orientation;
    this->velocity_ = velocity;
  };
};
