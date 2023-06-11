#include "../../include/utils/cone.hpp"

Cone::Cone(int id, float x, float y) : id(id), x(x), y(y) {}

int Cone::getId() const { return id; }

float Cone::getX() const { return x; }

void Cone::setX(float x) {
  this->x = x;
}

float Cone::getY() const { return y; }

void Cone::setY(float y) {
    this->y = y;
}

// std::string Cone::print() {
//   return "(" +std::to_string(x) + ", " + std::to_string(y) + ")";
// }

float Cone::getDistanceTo(Cone *dest) {
  return sqrt(pow(this->x - dest->getX(), 2) + pow(this->y - dest->getY(), 2));
}
