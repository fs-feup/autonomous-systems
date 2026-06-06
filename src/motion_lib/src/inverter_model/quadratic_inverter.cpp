#include "motion_lib/inverter_model/quadratic_inverter.hpp"

#include <algorithm>
#include <cmath>

double QuadraticInverter::apply_throttle_curve(double throttle_input) const {
  const double clamped_input = std::clamp(throttle_input, -1.0, 1.0);
  return std::copysign(clamped_input * clamped_input, clamped_input);
}
