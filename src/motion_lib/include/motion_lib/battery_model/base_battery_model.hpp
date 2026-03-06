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
   * @brief Calculate the maximum allowed current based on battery state and limits
   * @param current_request Current being requested (A)
   * @return Allowed current (A)
   */
  virtual double calculate_allowed_current(double current_request) const = 0;

  /**
   * @brief Get current being drawn from the battery
   * @return Current (A)
   */
  virtual double get_current() const = 0;

  /**
   * @brief Get current battery voltage with current state
   * @return Terminal voltage (V)
   */
  virtual double get_voltage() const = 0;

  /**
   * @brief Get current battery voltage under provided current draw
   * @return Terminal voltage (V)
   */
  virtual double get_voltage(double current_draw) const = 0;

  /**
   * @brief Get open circuit voltage
   * @return Open circuit voltage (V)
   */
  virtual double get_open_circuit_voltage() const = 0;

  /**
   * @brief Update battery state
   *
   * @param current_draw Current being drawn (A)
   * @param dt Time step (s)
   */
  virtual void update_state(double current_draw, double dt) = 0;

  /**
   * @brief Get current state of charge
   * @return SOC (0 to 1)
   */
  virtual double get_soc() const = 0;

  /**
   * @brief Reset battery
   */
  virtual void reset() = 0;
};
