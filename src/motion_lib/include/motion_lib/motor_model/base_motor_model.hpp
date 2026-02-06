#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"
#include "map.hpp"

/**
 * @brief Base class for motor models for electric powertrain
 */
class MotorModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  MotorModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Get motor efficiency at current state
   * @param torque Motor torque (Nm)
   * @param rpm Motor RPM
   * @return Efficiency (0 to 1)
   */
  virtual float getEfficiency(float torque, float rpm) const = 0;

  /**
   * @brief Get maximum torque available at current RPM
   * @param rpm Current motor RPM
   * @return Maximum available torque (Nm)
   */
  virtual float getMaxTorqueAtRPM(float rpm) const = 0;

  /**
   * @brief Update motor state
   * @param current_draw Current drawn (A)
   * @param dt Time step (s)
   */
  virtual void updateState(float current_draw, float dt) = 0;

  /**
   * @brief Reset motor state
   */
  virtual void reset() = 0;
};