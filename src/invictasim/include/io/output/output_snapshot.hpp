#pragma once

#include <Eigen/Core>

#include "common_lib/structures/wheels.hpp"

/**
 * @brief Vehicle model snapshot containing all vehicle state data to be published.
 */
struct VehicleModelSnapshot {
  // Tire data
  Eigen::Vector3d front_left_force = {0.0, 0.0, 0.0};
  Eigen::Vector3d front_right_force = {0.0, 0.0, 0.0};
  Eigen::Vector3d rear_left_force = {0.0, 0.0, 0.0};
  Eigen::Vector3d rear_right_force = {0.0, 0.0, 0.0};
  common_lib::structures::Wheels slip_ratio = {0.0, 0.0, 0.0, 0.0};
  common_lib::structures::Wheels slip_angle = {0.0, 0.0, 0.0, 0.0};

  // Motor data
  double motor_torque = 0.0;
  double motor_omega = 0.0;
  double motor_current = 0.0;
  double motor_thermal_state = 0.0;
  double motor_thermal_capacity = 0.0;

  // Battery data
  double battery_voltage = 0.0;
  double battery_open_circuit_voltage = 0.0;
  double battery_soc = 0.0;
  double battery_current = 0.0;

  // Differential data
  common_lib::structures::Wheels differential_torque = {0.0, 0.0, 0.0, 0.0};

  // Aero data
  double aero_drag = 0.0;
  double aero_downforce = 0.0;

  // Load transfer data
  common_lib::structures::Wheels vertical_load = {0.0, 0.0, 0.0, 0.0};

  // Status data
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double acceleration_x = 0.0;
  double acceleration_y = 0.0;
  double steering_angle = 0.0;
  common_lib::structures::Wheels wheel_speed = {0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief Snapshot of execution times for different components of the vehicle model
 */
struct ExecutionTimesSnapshot {
  double powertrain_ms = 0.0;
  double differential_ms = 0.0;
  double aero_ms = 0.0;
  double steering_ms = 0.0;
  double load_transfer_ms = 0.0;
  double tire_ms = 0.0;
  double total_step_ms = 0.0;
};

/**
 * @brief Snapshot of current simulator control input values.
 */
struct InputSnapshot {
  common_lib::structures::Wheels throttle = {0.0, 0.0, 0.0, 0.0};
  double steering = 0.0;
};