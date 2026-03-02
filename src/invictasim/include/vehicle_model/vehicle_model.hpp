#pragma once

#include <memory>
#include <string>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"
#include "config/config.hpp"

/**
 * @brief Vehicle state struct, needs to be extended
 */
struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  common_lib::structures::Wheels wheels_speed = {0.0, 0.0, 0.0, 0.0}; 
  common_lib::structures::Wheels wheels_torque = {0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief Vehicle model interface
 *
 * Defines a simplified interface that all vehicle models must implement. Certainly will be extended
 * during the vehicle model implementation.
 */
class VehicleModel {
protected:
  std::shared_ptr<InvictaSimParameters> simulator_parameters_;
  std::shared_ptr<VehicleState> state_;

public:
  /**
   * @brief Construct a new VehicleModel object
   */
  VehicleModel(const InvictaSimParameters& simulator_parameters)
      : simulator_parameters_(std::make_shared<InvictaSimParameters>(simulator_parameters)),
        state_(std::make_shared<VehicleState>()) {}

  /**
   * @brief Destroy the VehicleModel object
   */
  virtual ~VehicleModel() = default;

  // Core functions that all vehicle models must implement
  virtual void step(double dt, common_lib::structures::Wheels throttle, double angle) = 0;
  virtual void reset() = 0;

  // Common getters
  double get_position_x() const { return state_->x; }
  double get_position_y() const { return state_->y; }
  double get_position_z() const { return state_->z; }
  double get_roll() const { return state_->roll; }
  double get_pitch() const { return state_->pitch; }
  double get_yaw() const { return state_->yaw; }
  double get_velocity_x() const { return state_->vx; }
  double get_velocity_y() const { return state_->vy; }
  double get_velocity_z() const { return state_->vz; }
  common_lib::structures::Wheels get_wheels_speed() const { return state_->wheels_speed; }
  common_lib::structures::Wheels get_wheels_torque() const { return state_->wheels_torque; }

  // Specific getters
  virtual double get_motor_torque() const = 0;
  virtual double get_battery_current() const = 0;
  virtual double get_battery_voltage() const = 0;
  virtual double get_battery_soc() const = 0;
  virtual std::string get_model_name() const = 0;

  // Essential model setters
  void set_position(double x, double y, double z) {
    state_->x = x;
    state_->y = y;
    state_->z = z;
  }
  void set_velocity(double vx, double vy, double vz) {
    state_->vx = vx;
    state_->vy = vy;
    state_->vz = vz;
  }
  void set_orientation(double roll, double pitch, double yaw) {
    state_->roll = roll;
    state_->pitch = pitch;
    state_->yaw = yaw;
  }
  void set_wheels_speed(const common_lib::structures::Wheels& wheels_speed) {
    state_->wheels_speed = wheels_speed;
  }
  void set_wheels_torque(const common_lib::structures::Wheels& wheels_torque) {
    state_->wheels_torque = wheels_torque;
  }
};
