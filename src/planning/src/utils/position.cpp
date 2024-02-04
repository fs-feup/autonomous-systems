#include "../../include/utils/position.hpp"

Position::Position(float x, float y, float v) : x(x), y(y), v(v) {}

float Position::getX() const { return x; }

float Position::getY() const { return y; }

float Position::getV() const { return v; }

float Position::getDistanceTo(Position *dest) {
  return sqrt(pow(this->x - dest->getX(), 2) + pow(this->y - dest->getY(), 2));
}

bool Position::operator==(const Position &p) const {
  return this->x == p.getX() && this->y == p.getY();
}