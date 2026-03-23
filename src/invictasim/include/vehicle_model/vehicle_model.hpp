#pragma once

#include <Eigen/Core>
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
  double vz = 0.0;  // In case we fly
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double angular_speed = 0.0;
  double steering_angle = 0.0;
  double angular_velocity = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  common_lib::structures::Wheels wheels_speed = {0.0, 0.0, 0.0, 0.0};  // rad/s
  common_lib::structures::Wheels wheels_torque = {0.0, 0.0, 0.0, 0.0};
  common_lib::structures::Wheels wheels_vertical_load = {0.0, 0.0, 0.0, 0.0};
  common_lib::structures::Wheels wheels_slip_ratio = {0.0, 0.0, 0.0, 0.0};
  common_lib::structures::Wheels wheels_slip_angle = {0.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d front_left_forces = {0.0, 0.0, 0.0};  // Fx, Fy, Fz
  Eigen::Vector3d front_right_forces = {0.0, 0.0, 0.0};
  Eigen::Vector3d rear_left_forces = {0.0, 0.0, 0.0};
  Eigen::Vector3d rear_right_forces = {0.0, 0.0, 0.0};
  double aero_drag = 0.0;
  double aero_downforce = 0.0;
  double motor_torque = 0.0;
  double motor_omega = 0.0;
  double motor_current = 0.0;
  double motor_thermal_state = 0.0;
  double motor_thermal_capacity = 0.0;
  double battery_voltage = 0.0;
  double battery_soc = 0.0;
  double battery_current = 0.0;
  double battery_open_circuit_voltage = 0.0;
  double total_force_x = 0.0;
  double total_force_y = 0.0;
  double moment_fy = 0.0;
  double moment_fx = 0.0;
  double self_aligning_moment = 0.0;
  double total_torque_z = 0.0;
};

/**4
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
  double get_yaw_rate() const { return state_->yaw_rate; }
  double get_velocity_x() const { return state_->vx; }
  double get_velocity_y() const { return state_->vy; }
  double get_velocity_z() const { return state_->vz; }
  double get_acceleration_x() const { return state_->ax; }
  double get_acceleration_y() const { return state_->ay; }
  common_lib::structures::Wheels get_wheels_speed() const { return state_->wheels_speed; }
  common_lib::structures::Wheels get_wheels_torque() const { return state_->wheels_torque; }
  common_lib::structures::Wheels get_wheels_vertical_load() const {
    return state_->wheels_vertical_load;
  }
  common_lib::structures::Wheels get_wheels_slip_ratio() const { return state_->wheels_slip_ratio; }
  common_lib::structures::Wheels get_wheels_slip_angle() const { return state_->wheels_slip_angle; }
  Eigen::Vector3d get_front_left_forces() const { return state_->front_left_forces; }
  Eigen::Vector3d get_front_right_forces() const { return state_->front_right_forces; }
  Eigen::Vector3d get_rear_left_forces() const { return state_->rear_left_forces; }
  Eigen::Vector3d get_rear_right_forces() const { return state_->rear_right_forces; }
  double get_aero_drag() const { return state_->aero_drag; }
  double get_aero_downforce() const { return state_->aero_downforce; }
  double get_motor_omega() const { return state_->motor_omega; }
  double get_motor_current() const { return state_->motor_current; }
  double get_motor_thermal_state() const { return state_->motor_thermal_state; }
  double get_motor_thermal_capacity() const { return state_->motor_thermal_capacity; }
  double get_battery_open_circuit_voltage() const { return state_->battery_open_circuit_voltage; }
  double get_steering_angle() const { return state_->steering_angle; }
  double get_total_force_x() const { return state_->total_force_x; }
  double get_total_force_y() const { return state_->total_force_y; }
  double get_moment_fy() const { return state_->moment_fy; }
  double get_moment_fx() const { return state_->moment_fx; }
  double get_self_aligning_moment() const { return state_->self_aligning_moment; }
  double get_total_torque_z() const { return state_->total_torque_z; }

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
