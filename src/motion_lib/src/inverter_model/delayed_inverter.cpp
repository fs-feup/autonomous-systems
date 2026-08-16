#include "motion_lib/inverter_model/delayed_inverter.hpp"

#include <algorithm>
#include <cmath>

double DelayedInverter::calculate_inverter_throttle(double throttle_input, double dt) {
  current_time_s_ += dt;

  const double clamped_input = std::clamp(throttle_input, -1.0, 1.0);
  pending_commands_.push_back({current_time_s_ + delay_for_input(clamped_input), clamped_input});

  while (!pending_commands_.empty() && pending_commands_.front().first <= current_time_s_) {
    output_throttle_input_ = pending_commands_.front().second;
    pending_commands_.pop_front();
  }

  return output_throttle_input_;
}

void DelayedInverter::reset() {
  current_time_s_ = 0.0;
  output_throttle_input_ = 0.0;
  pending_commands_.clear();
}

double DelayedInverter::delay_for_input(double throttle_input) const {
  const auto& inverter_parameters = car_parameters_->inverter_parameters;
  double delay_ms = inverter_parameters->coast_delay_ms;
  if (throttle_input > 0.0) {
    delay_ms = inverter_parameters->acceleration_delay_ms;
  } else if (throttle_input < 0.0) {
    delay_ms = inverter_parameters->regen_braking_delay_ms;
  }

  return std::abs(throttle_input) * delay_ms / 1000.0;
}
