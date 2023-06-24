#include "../../include/utils/position.hpp"

Position::Position(float x, float y) : x(x), y(y) {}

float Position::getX() const { return x; }

/*void Position::setX(float x) {
    Position::x = x;
}*/

float Position::getY() const { return y; }

/*void Position::setY(float y) {
    Position::y = y;
}*/

float Position::getDistanceTo(Position *dest) {
  return sqrt(pow(this->x - dest->getX(), 2) + pow(this->y - dest->getY(), 2));
}

bool Position::operator==(const Position& p) const{
  if(this-> x == p.getX() && this-> y == p.getY() ){
    return true;
  }
  return false;
}