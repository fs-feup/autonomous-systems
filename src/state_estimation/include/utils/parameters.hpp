#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"

struct SEParameters {
  std::string adapter_;                 // Adapter name to be used, e.g. "pacsim", "vehile"
  std::string estimation_method_;       // Used to choose between different state estimation methods
  std::string process_model_name_;      // Used to choose between different process models
  std::string observation_model_name_;  // Used to choose between different state estimation
                                        // observation models

  double state_estimation_freq_;  // The rate at which the state estimation should run (in Hz)
  int prediction_threads_;  // OpenMP threads for the sigma-point prediction loop (scoped to this node)

  std::string vehicle_model_name_;         // Used to choose between different vehicle models
  std::string load_transfer_model_name_;   // Used to choose between different load transfer models
  std::string aero_model_name_;            // Choose between different aero models
  std::string steering_model_name_;        // Choose between different steering models
  std::string steering_motor_model_name_;  // Choose between different steering motor models
  std::string transmission_model_name_;    // Choose between different transmission models
  std::string tire_model_name_;            // Choose between different tire models

  double velocity_x_process_noise_;      // Process noise for the velocity x predicted state
  double velocity_y_process_noise_;      // Process noise for the velocity y predicted state
  double yaw_rate_process_noise_;        // Process noise for the yaw rate predicted state
  double acceleration_x_process_noise_;  // Process noise for the acceleration x predicted state
  double acceleration_y_process_noise_;  // Process noise for the acceleration y predicted state
  double steering_angle_process_noise_;  // Process noise for the steering angle predicted state
  double wheel_speed_process_noise_;     // Process noise for the wheel speed predicted state

  double imu_acceleration_x_noise_;  // Noise to be added to the IMU acceleration measurements
  double imu_acceleration_y_noise_;  // Noise to be added to the IMU acceleration measurements
  double imu_rotational_noise_;      // Noise to be added to the IMU rotational state measurements
  double wheel_speed_noise_;         // Noise to be added to the wheel speed measurements
  double motor_rpm_noise_;           // Noise to be added to the motor rpm measurements
  double steering_angle_noise_;      // Noise to be added to the steering angle measurements

  // Paired (state-based / kinematic) measurement-noise factors. Wheel and motor sensors are
  // observed by two correlated rows sharing the same physical measurement: a state-based row
  // and a kinematic row. Each factor multiplies the base sensor variance (var).
  // R_pair = [[var*(1 + state_factor), cross_factor*var      ],
  //           [cross_factor*var,       var*(1 + kinematic_factor)]]
  double state_model_noise_factor_;      // Extra variance on the state-based row
  double kinematic_model_noise_factor_;  // Extra variance on the kinematic row
  double cross_noise_factor_;            // Shared measurement covariance between the two rows

  std::shared_ptr<common_lib::car_parameters::CarParameters>
      car_parameters_;  // Car parameters to be used in the process and observation models

  double mean_weight_;  // UKF mean weight parameter
  double alpha_;        // UKF alpha parameter
  double beta_;         // UKF beta parameter
  double kappa_;        // UKF kappa parameter

  bool publish_vm_debug_info_;  // Publish debug information about the vehicle model
  bool publish_exec_times_;     // Publish execution time statistics for filter stages
  bool publish_sensor_health_;  // Publish per-sensor health from the Overseer

  bool use_fresh_measurements_only_;  // Only use rows whose sensor delivered a new sample since
                                      // the previous filter cycle (avoids re-applying stale data)
  double innovation_gate_;  // Normalized-innovation-squared threshold above which a measurement
                            // row is soft-rejected (variance inflated). 0 disables the gate.

  // --- Sensor fault detection (Overseer) ---
  bool sensor_fault_detection_enabled_;  // Master switch: when false, health never affects R
  double sensor_faulty_noise_factor_;    // Bounded R multiplier for FAULTY rows (DEAD are dropped)
  double sensor_staleness_miss_tolerance_;  // Missed sample periods of silence before DEAD
  double imu_frequency_;                    // Expected IMU publish rate (Hz)
  double wheel_speed_frequency_;            // Expected wheel encoder publish rate (Hz)
  double steering_frequency_;               // Expected steering sensor publish rate (Hz)
  double imu_accel_range_;                  // Max |acceleration| accepted from the IMU (m/s^2)
  double imu_accel_max_rate_of_change_;  // Max |d(acceleration)/dt| accepted from the IMU (m/s^3)
  double imu_yaw_range_;                 // Max |yaw rate| accepted from the IMU (rad/s)
  double imu_yaw_max_rate_of_change_;    // Max |d(yaw rate)/dt| accepted from the IMU (rad/s^2)
  double wheel_speed_range_;             // Max |wheel speed| accepted from an encoder (rpm)
  double
      wheel_speed_max_rate_of_change_;   // Max |d(wheel speed)/dt| accepted from an encoder (rpm/s)
  double steering_range_;                // Max |steering angle| accepted (rad)
  double steering_max_rate_of_change_;   // Max |d(steering angle)/dt| accepted (rad/s)
  double motor_frequency_;               // Expected motor rpm publish rate (Hz)
  double motor_rpm_range_;               // Max |motor rpm| accepted (rpm)
  double motor_rpm_max_rate_of_change_;  // Max |d(motor rpm)/dt| accepted (rpm/s)

  /**
   * @brief Load the configuration for the State Estimation node from YAML file
   *
   * @return std::string adapter_name
   */
  std::string load_config();
};
