#include "utils/formulas.hpp"

#include <cmath>

double normalize_angle(double angle) {
  while (angle < 0.0) {
    angle += 2 * M_PI;
  }
  while (angle >= 2 * M_PI) {
    angle -= 2 * M_PI;
  }
  return angle;
}