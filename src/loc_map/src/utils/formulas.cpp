#include <cmath>

#include "utils/formulas.hpp"

double sin_in_degrees(double angle) {
  return sin(angle * M_PI / 180.0);
}

double cos_in_degrees(double angle) {
  return cos(angle * M_PI / 180.0);
}

double normalize_angle(double angle) {
  while (angle < 0.0) {
    angle += 360.0;
  }
  while (angle >= 360.0) {
    angle -= 360.0;
  }
  return angle;
}