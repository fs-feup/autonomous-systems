#pragma once

#include "base_motor_model.hpp"
#include "common_lib/car_parameters/car_parameters.hpp"

/**
 * @brief Map based motor model for electric powertrain
 */
class MapBasedMotor : public MotorModel {
public:
  MapBasedMotor(const common_lib::car_parameters::CarParameters& car_parameters);

  float get_efficiency(float torque, float rpm) const override;

  float get_max_torque_at_rpm(float rpm) const override;

  void update_state(float current_draw, float torque, float dt) override;

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
  float interpolate_from_map(const std::map<float, float>& map, float key) const;
};