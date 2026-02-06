#pragma once

#include "base_battery_model.hpp"

/**
 * @brief Equivalent circuit model of a battery
 */
class EquivalentCircuitBattery : public BatteryModel {
public:
  explicit EquivalentCircuitBattery(
      const common_lib::car_parameters::CarParameters& car_parameters);

  std::tuple<float, float> calculateCurrentForPower(float electrical_power_req) const override;

  float getVoltage() const override;

  float getOpenCircuitVoltage() const override;

  void updateState(float current_draw, float dt) override;

  float getSoC() const override;

  void reset() override;

private:
  float soc_;               // Current state of charge (0-1)
  float thermal_capacity_;  // K = (I_peak² - I_cont²) × T_max_peak (A²·s)
  float thermal_state_;     // Current accumulated heat (A²·s), range [0, thermal_capacity_]
  float current_;           // Current being drawn (A)

  /**
   * @brief Calculate heat generation from I²R losses
   */
  float calculateHeatGeneration(float current_draw) const;

  /**
   * @brief Generic linear interpolation from a map
   */
  float interpolateFromMap(const std::map<float, float>& map, float key) const;
};
