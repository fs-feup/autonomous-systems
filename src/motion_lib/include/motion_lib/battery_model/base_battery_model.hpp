#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"

/**
 * @brief Base class for battery models
 */
class BatteryModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  BatteryModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Calculate current and available power for given electrical power request
   *
   * @param electrical_power_req Requested electrical power (W)
   * @return std::tuple<float, float> Feasible current (A) and available electrical power (W)
   */
  virtual std::tuple<float, float> calculate_current_for_power(
      float electrical_power_req) const = 0;

  /**
   * @brief Get current being drawn from the battery
   * @return Current (A)
   */
  virtual float get_current() const = 0;

  /**
   * @brief Get current battery voltage with current state
   * @return Terminal voltage (V)
   */
  virtual float get_voltage() const = 0;

  /**
   * @brief Get current battery voltage under provided current draw
   * @return Terminal voltage (V)
   */
  virtual float get_voltage(float current_draw) const = 0;

  /**
   * @brief Get open circuit voltage
   * @return Open circuit voltage (V)
   */
  virtual float get_open_circuit_voltage() const = 0;

  /**
   * @brief Update battery state
   *
   * @param current_draw Current being drawn (A)
   * @param dt Time step (s)
   */
  virtual void update_state(float current_draw, float dt) = 0;

  /**
   * @brief Get current state of charge
   * @return SOC (0 to 1)
   */
  virtual float get_soc() const = 0;

  /**
   * @brief Reset battery
   */
  virtual void reset() = 0;
};
