#pragma once

#include <Eigen/Core>
#include <vector>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/structures/wheels.hpp"

/**
 * @brief Vehicle model snapshot containing all vehicle state data to be published.
 */
struct VehicleModelSnapshot {
  // Tire data
  Eigen::Vector4d front_left_force = {0.0, 0.0, 0.0, 0.0};
  Eigen::Vector4d front_right_force = {0.0, 0.0, 0.0, 0.0};
  Eigen::Vector4d rear_left_force = {0.0, 0.0, 0.0, 0.0};
  Eigen::Vector4d rear_right_force = {0.0, 0.0, 0.0, 0.0};
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

  // Transmission data
  common_lib::structures::Wheels transmission_torque = {0.0, 0.0, 0.0, 0.0};

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
  double transmission_ms = 0.0;
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
  std::vector<common_lib::structures::Cone> external_slam_cones = {};
  std::vector<common_lib::structures::Cone> external_perception_cones = {};
  std::vector<common_lib::structures::PathPoint> external_path_points = {};
};

/**
 * @brief Snapshot of the track map, containing both the ground truth cone positions and simulated
 * slam map
 */
struct MapSnapshot {
  std::vector<common_lib::structures::Cone> ground_truth = {};
  std::vector<common_lib::structures::Cone> simulated_slam_map = {};
  std::vector<common_lib::structures::Cone> perception_cones = {};
  std::vector<common_lib::structures::Cone> recently_hit_cones = {};
  double perception_exec_time_ms = 0.0;
};

/**
 * @brief Snapshot of accumulated simulator statistics.
 */
struct StatisticsSnapshot {
  // Completed lap summary
  int lap_counter = 0;
  double last_lap_time = 0.0;
  int cones_hit = 0;
  double total_lap_time = 0.0;
  double best_lap_time = 0.0;
  double completed_lap_average_velocity = 0.0;
  double completed_lap_max_velocity = 0.0;
  double completed_lap_average_tracking_error = 0.0;
  double completed_lap_max_tracking_error = 0.0;
  double completed_lap_average_velocity_error = 0.0;
  double completed_lap_max_velocity_error = 0.0;

  // Current lap
  double current_lap_time = 0.0;
  int current_lap_cones_hit = 0;

  // Control statistics
  double current_velocity = 0.0;
  double objective_velocity = 0.0;
  double tracking_cross_track_error = 0.0;
  double velocity_error = 0.0;
};

/**
 * @brief Snapshot of sensor data for publishing simulated IMU, WSS, steering angle sensor and
 * resolver
 */
struct SensorsSnapshot {
  Eigen::Vector3d free_acceleration = {0.0, 0.0, 0.0};
  Eigen::Vector3d angular_velocity = {0.0, 0.0, 0.0};
  common_lib::structures::Wheels wheel_rpm = {0.0, 0.0, 0.0, 0.0};
  double steering_angle = 0.0;
  double motor_rpm = 0.0;
};

/**
 * @brief Snapshot of the vehicle's state for publishing the current pose and operational status,
 * used for state estimation / SLAM / planning compatibility topics.
 */
struct VehicleStateSnapshot {
  // Pose from state_estimation
  common_lib::structures::Position position = {0.0, 0.0};
  double yaw = 0.0;
  std::vector<double> pose_covariance = std::vector<double>(9, 0.0);

  // Velocities from velocity_estimation
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double yaw_rate = 0.0;
  std::vector<double> velocity_covariance = std::vector<double>(9, 0.0);

  // Operational status
  bool go_signal = false;
  common_lib::competition_logic::Mission mission = common_lib::competition_logic::Mission::NONE;
};
