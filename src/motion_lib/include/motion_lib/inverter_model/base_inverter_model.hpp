#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"

class InverterModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  explicit InverterModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  virtual ~InverterModel() = default;

  virtual double calculate_inverter_throttle(double throttle_input, double dt) = 0;

  double get_efficiency() const{
    return car_parameters_->inverter_parameters->efficiency;
  }

  double get_max_phase_current() const {
    return car_parameters_->inverter_parameters->max_phase_current;
  }

  virtual void reset() = 0;
};
