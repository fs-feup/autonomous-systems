#pragma once

#include <memory>
#include <string>

/**
 * @brief Vehicle model interface
 *
 * Defines a simplified interface that all vehicle models must implement. Certainly will be extended
 * during the vehicle model implementation.
 */
class VehicleModel {
public:
  virtual ~VehicleModel() = default;
  virtual void step(double dt) = 0;
  virtual void reset() = 0;

  // Essential state getters
  virtual double get_position_x() const = 0;
  virtual double get_position_y() const = 0;
  virtual double get_yaw() const = 0;
  virtual double get_velocity_x() const = 0;

  // Essential state setters
  virtual void set_position(double x, double y, double yaw) = 0;
  virtual void set_velocity(double vx) = 0;

  // Control inputs
  virtual void set_steering(double angle) = 0;
  virtual void set_throttle(double throttle) = 0;

  virtual std::string get_model_name() const = 0;
};
