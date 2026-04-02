#pragma once

#include "base_motor_model.hpp"
#include "common_lib/car_parameters/car_parameters.hpp"

/**
 * @brief Map based motor model for electric powertrain
 */
class MapBasedMotor : public MotorModel {
public:
  MapBasedMotor(const common_lib::car_parameters::CarParameters& car_parameters);

  double get_efficiency(double torque, double rpm) const override;

  double get_max_torque_at_rpm(double rpm) const override;

  void update_state(double current_draw, double torque, double dt) override;

  void reset() override;

  double get_torque() const override;

  double get_current() const override;

  double get_thermal_state() const override;

  double get_thermal_capacity() const override;

private:
  double torque_;            // Torque being applied to the motor after battery influence (Nm)
  double current_;           // Current being drawn (A)
  double thermal_capacity_;  // K = (I_peak² - I_cont²) × T_max_peak (A²·s)
  double thermal_state_;     // Current accumulated heat (A²·s), range [0, thermal_capacity_]

  /**
   * @brief Generic linear interpolation from a map
   */
  double interpolate_from_map(const std::map<double, double>& map, double key) const;
};