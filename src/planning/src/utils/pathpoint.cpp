#include "../../include/utils/pathpoint.hpp"

PathPoint::PathPoint(float x, float y, float v) : x(x), y(y), v(v) {}

float PathPoint::getX() const { return x; }

float PathPoint::getY() const { return y; }

float PathPoint::getV() const { return v; }

float PathPoint::getDistanceTo(PathPoint *dest) {
  return sqrt(pow(this->x - dest->getX(), 2) + pow(this->y - dest->getY(), 2));
}

bool PathPoint::operator==(const PathPoint &p) const {
  return this->x == p.getX() && this->y == p.getY();
}