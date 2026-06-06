#include "motion_lib/inverter_model/linear_inverter.hpp"

#include <algorithm>

double LinearInverter::apply_throttle_curve(double throttle_input) const {
  return std::clamp(throttle_input, -1.0, 1.0);
}
