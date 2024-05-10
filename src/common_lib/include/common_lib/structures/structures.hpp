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
  Point position;
  double heading;
  double orientation;
  double velocity;

  // Default constructor
  Pose() {
    position = Point();
    heading = 0.0;
    orientation = 0.0;
    velocity = 0.0;
  }

  // Constructor
  Pose(Point position, double heading, double orientation, double velocity) {
    this->position = position;
    this->heading = heading;
    this->orientation = orientation;
    this->velocity = velocity;
  };
};
