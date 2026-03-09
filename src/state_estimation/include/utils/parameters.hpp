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

  std::string load_transfer_model_name_;   // Used to choose between different load transfer models
  std::string aero_model_name_;            // Choose between different aero models
  std::string steering_model_name_;        // Choose between different steering models
  std::string steering_motor_model_name_;  // Choose between different steering motor models
  std::string differential_model_name_;    // Choose between different differential models
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

  std::shared_ptr<common_lib::car_parameters::CarParameters>
      car_parameters_;  // Car parameters to be used in the process and observation models

  double alpha_;  // UKF alpha parameter
  double kappa_;  // UKF kappa parameter

  /**
   * @brief Load the configuration for the State Estimation node from YAML file
   *
   * @return std::string adapter_name
   */
  std::string load_config();
};
