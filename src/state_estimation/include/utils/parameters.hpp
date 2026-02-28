#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"

struct SEParameters {
  std::string estimation_method_;       // Used to choose between different state estimation methods
  std::string process_model_name_;      // Used to choose between different process models
  std::string observation_model_name_;  // Used to choose between different state estimation
                                        // observation models

  double state_estimation_freq_;  // The rate at which the state estimation should run (in Hz)

  double velocity_x_process_noise_;      // Process noise for the velocity x predicted state
  double velocity_y_process_noise_;      // Process noise for the velocity y predicted state
  double yaw_rate_process_noise_;        // Process noise for the yaw rate predicted state
  double acceleration_x_process_noise_;  // Process noise for the acceleration x predicted state
  double acceleration_y_process_noise_;  // Process noise for the acceleration y predicted state
  double steering_angle_process_noise_;  // Process noise for the steering angle predicted state
  double wheel_speed_process_noise_;     // Process noise for the wheel speed predicted state

  double imu_acceleration_noise_;  // Noise to be added to the IMU acceleration measurements
  double imu_rotational_noise_;    // Noise to be added to the IMU rotational state measurements
  double wheel_speed_noise_;       // Noise to be added to the wheel speed measurements
  double motor_rpm_noise_;         // Noise to be added to the motor rpm measurements
  double steering_angle_noise_;    // Noise to be added to the steering angle measurements

  common_lib::car_parameters::CarParameters car_parameters_;

  double alpha;
  double kappa;

  /**
   * @brief Load the configuration for the State Estimation node from YAML file
   *
   * @return std::string adapter_name
   */
  std::string load_config();
};
