#pragma once

#include "base_motor_model.hpp"
#include "common_lib/car_parameters/car_parameters.hpp"

/**
 * @brief Map based motor model for electric powertrain
 */
class MapBasedMotor : public MotorModel {
public:
  MapBasedMotor(const common_lib::car_parameters::CarParameters& car_parameters);

  float getEfficiency(float torque, float rpm) const override;

  float getMaxTorqueAtRPM(float rpm) const override;

  void updateState(float current_draw, float torque, float dt) override;

  void reset() override;

  float get_torque() const override;

private:
  float torque_;            // Torque being applied to the motor after battery influence (Nm)
  float current_;           // Current being drawn (A)
  float thermal_capacity_;  // K = (I_peak² - I_cont²) × T_max_peak (A²·s)
  float thermal_state_;     // Current accumulated heat (A²·s), range [0, thermal_capacity_]

  /**
   * @brief Generic linear interpolation from a map
   */
  float interpolateFromMap(const std::map<float, float>& map, float key) const;
};