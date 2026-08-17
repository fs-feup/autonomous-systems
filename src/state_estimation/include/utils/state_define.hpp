#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/structures/wheels.hpp"

#define StateSize 10
#define State Eigen::Matrix<double, StateSize, 1>

#define VX 0
#define VY 1
#define YAW_RATE 2
#define AX 3
#define AY 4
#define ST_ANGLE 5
#define FL_WHEEL_SPEED 6
#define FR_WHEEL_SPEED 7
#define RL_WHEEL_SPEED 8
#define RR_WHEEL_SPEED 9

/**
 * @brief Full Vehicle state struct
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
  Eigen::Vector4d front_left_forces = {0.0, 0.0, 0.0, 0.0};  // Fx, Fy, My, Mz
  Eigen::Vector4d front_right_forces = {0.0, 0.0, 0.0, 0.0};
  Eigen::Vector4d rear_left_forces = {0.0, 0.0, 0.0, 0.0};
  Eigen::Vector4d rear_right_forces = {0.0, 0.0, 0.0, 0.0};
  double aero_drag = 0.0;
  double aero_downforce = 0.0;
  double battery_open_circuit_voltage = 0.0;
  double total_force_x = 0.0;
  double total_force_y = 0.0;
  double moment_fy = 0.0;
  double moment_fx = 0.0;
  double self_aligning_moment = 0.0;
  double total_torque_z = 0.0;
};
