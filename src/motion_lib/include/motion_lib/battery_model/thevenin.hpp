#pragma once

#include "base_battery_model.hpp"

/**
 * @brief Equivalent circuit model of a battery
 */
class Thevenin : public BatteryModel {
public:
  explicit Thevenin(const common_lib::car_parameters::CarParameters& car_parameters);

  std::tuple<float, float> calculate_current_for_power(float electrical_power_req) const override;

  float get_current() const override;

  float get_voltage() const override;

  float get_voltage(float current_draw) const override;

  float get_open_circuit_voltage() const override;

  void update_state(float current_draw, float dt) override;

  float get_soc() const override;

  void reset() override;

private:
  float soc_;      // Current state of charge (0-1)
  float v_rc_;     // Voltage across RC branch (V)
  float current_;  // Current being drawn (A)

  float solve_polinomial5(const std::vector<float>& coeffs, float x) const;

  float get_cell_ocv(float soc) const;

  float get_cell_r0(float soc) const;

  float get_cell_r1(float soc) const;

  float get_cell_c1(float soc) const;
};
