#pragma once

#include "base_battery_model.hpp"

/**
 * @brief Equivalent circuit model of a battery
 */
class Thevenin : public BatteryModel {
public:
  explicit Thevenin(const common_lib::car_parameters::CarParameters& car_parameters);

  double calculate_allowed_current(double current_draw) const override;

  double get_current() const override;

  double get_voltage() const override;

  double get_voltage(double current_draw) const override;

  double get_open_circuit_voltage() const override;

  void update_state(double current_draw, double dt) override;

  double get_soc() const override;

  void reset() override;

private:
  double soc_;      // Current state of charge (0-1)
  double v_rc_;     // Voltage across RC branch (V)
  double current_;  // Current being drawn (A)

  double solve_polinomial5(const std::vector<double>& coeffs, double x) const;

  double get_cell_ocv(double soc) const;

  double get_cell_r0(double soc) const;

  double get_cell_r1(double soc) const;

  double get_cell_c1(double soc) const;
};
