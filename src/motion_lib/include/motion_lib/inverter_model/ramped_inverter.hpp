#pragma once

#include "base_inverter_model.hpp"

/**
 * @brief Slew-rate limited inverter.
 *
 * Reproduces how a Bamocar (and motor controllers generally) applies a torque
 * command: the setpoint is ramped towards the request at a configured rate rather
 * than applied instantly. The ramp times are quoted for the full command range, so
 * the rate is their reciprocal.
 */
class RampedInverter : public InverterModel {
public:
  explicit RampedInverter(const common_lib::car_parameters::CarParameters& car_parameters)
      : InverterModel(car_parameters) {}

  double calculate_inverter_throttle(double throttle_input, double dt) override;

  void reset() override;

private:
  double output_throttle_input_ = 0.0;
};
