#pragma once

#include "motion_lib/transmission_model/base_transmission_model.hpp"

class SalisburyDifferential : public TransmissionModel {
 public:
  SalisburyDifferential(const common_lib::car_parameters::CarParameters& params)
      : TransmissionModel(params) {}

  virtual ~SalisburyDifferential() = default;

  double calculate_motor_omega(
      const common_lib::structures::Wheels& wheel_speeds) const override;

  common_lib::structures::Wheels calculate_wheel_torques(
      double input_torque, const common_lib::structures::Wheels& wheel_speeds) const override;
};
