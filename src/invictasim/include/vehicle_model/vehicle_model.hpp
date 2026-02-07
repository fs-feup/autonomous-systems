#pragma once

#include <memory>
#include <string>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "config/config.hpp"

/**
 * @brief Vehicle model interface
 *
 * Defines a simplified interface that all vehicle models must implement. Certainly will be extended
 * during the vehicle model implementation.
 */
class VehicleModel {
protected:
  std::shared_ptr<InvictaSimParameters> simulator_parameters_;

public:
  VehicleModel(const InvictaSimParameters& simulator_parameters)
      : simulator_parameters_(std::make_shared<InvictaSimParameters>(simulator_parameters)) {}
  virtual ~VehicleModel() = default;

  // Core functions
  virtual void step(double dt, double angle, double throttle) = 0;
  virtual void reset() = 0;

  // Essential state getters
  virtual double get_position_x() const = 0;
  virtual double get_position_y() const = 0;
  virtual double get_yaw() const = 0;
  virtual double get_velocity_x() const = 0;

  // Essential state setters
  virtual void set_position(double x, double y, double yaw) = 0;
  virtual void set_velocity(double vx) = 0;

  virtual std::string get_model_name() const = 0;
};
