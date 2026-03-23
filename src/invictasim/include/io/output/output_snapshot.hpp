#pragma once

#include <Eigen/Core>

#include "common_lib/structures/wheels.hpp"

struct TireSnapshot {
  Eigen::Vector3d front_left_force = {0.0, 0.0, 0.0};
  Eigen::Vector3d front_right_force = {0.0, 0.0, 0.0};
  Eigen::Vector3d rear_left_force = {0.0, 0.0, 0.0};
  Eigen::Vector3d rear_right_force = {0.0, 0.0, 0.0};
  common_lib::structures::Wheels slip_ratio = {0.0, 0.0, 0.0, 0.0};
  common_lib::structures::Wheels slip_angle = {0.0, 0.0, 0.0, 0.0};
};

struct PowertrainSnapshot {
  double motor_torque = 0.0;
  double motor_omega = 0.0;
  double motor_current = 0.0;
  double motor_thermal_state = 0.0;
  double motor_thermal_capacity = 0.0;
  double battery_voltage = 0.0;
  double battery_open_circuit_voltage = 0.0;
  double battery_soc = 0.0;
  double battery_current = 0.0;
  common_lib::structures::Wheels differential_torque = {0.0, 0.0, 0.0, 0.0};
};

struct AeroSnapshot {
  double drag = 0.0;
  double downforce = 0.0;
};

struct LoadSnapshot {
  common_lib::structures::Wheels vertical_load = {0.0, 0.0, 0.0, 0.0};
};

struct StatusSnapshot {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double velocity_z = 0.0;
  double acceleration_x = 0.0;
  double acceleration_y = 0.0;
  double steering_angle = 0.0;
  double total_force_x = 0.0;
  double total_force_y = 0.0;
  double moment_fy = 0.0;
  double moment_fx = 0.0;
  double self_aligning_moment = 0.0;
  double total_torque_z = 0.0;
  common_lib::structures::Wheels wheel_speed = {0.0, 0.0, 0.0, 0.0};
};

struct AggregateOutputSnapshot {
  double sim_time = 0.0;
  TireSnapshot tire;
  PowertrainSnapshot powertrain;
  AeroSnapshot aero;
  LoadSnapshot load;
  StatusSnapshot status;
};