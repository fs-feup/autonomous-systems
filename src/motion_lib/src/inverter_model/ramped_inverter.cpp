#include "motion_lib/inverter_model/ramped_inverter.hpp"

#include <algorithm>
#include <cmath>

double RampedInverter::calculate_inverter_throttle(double throttle_input, double dt) {
  const double target = std::clamp(throttle_input, -1.0, 1.0);
  const double error = target - output_throttle_input_;
  if (error == 0.0 || dt <= 0.0) {
    return output_throttle_input_;
  }

  // Rising demand uses the acceleration ramp, falling demand the braking ramp.
  const auto& inverter = car_parameters_->inverter_parameters;
  const double ramp_ms =
      error > 0.0 ? inverter->acceleration_ramp_ms : inverter->regen_braking_ramp_ms;
  if (ramp_ms <= 0.0) {
    output_throttle_input_ = target;
    return output_throttle_input_;
  }

  const double max_step = dt / (ramp_ms / 1000.0);
  output_throttle_input_ += std::copysign(std::min(max_step, std::abs(error)), error);
  return output_throttle_input_;
}

void RampedInverter::reset() { output_throttle_input_ = 0.0; }
