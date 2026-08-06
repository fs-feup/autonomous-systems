#pragma once

#include <deque>
#include <utility>

#include "base_inverter_model.hpp"

class DelayedInverter : public InverterModel {
public:
  explicit DelayedInverter(const common_lib::car_parameters::CarParameters& car_parameters)
      : InverterModel(car_parameters) {}

  double calculate_inverter_throttle(double throttle_input, double dt) override;

  void reset() override;

private:
  double current_time_s_ = 0.0;
  double output_throttle_input_ = 0.0;
  std::deque<std::pair<double, double>> pending_commands_;

  double delay_for_input(double throttle_input) const;
};
