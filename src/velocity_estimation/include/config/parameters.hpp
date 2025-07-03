#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"

struct VEParameters {
  std::string _estimation_method_;  // Used to choose between different velocity estimation methods
  std::string _ve_observation_model_name_;  // Used to choose between different S2V models
  std::string _process_model_name_;         // Used to choose between different process models
  double imu_acceleration_noise_;  // Noise to be added to the IMU acceleration measurements
  double imu_rotational_noise_;    // Noise to be added to the IMU rotational velocity measurements
  double angular_velocity_process_noise_;  // Process noise for the angular velocity, represents
                                           // expected variation
  double wheel_speed_noise_;               // Noise to be added to the wheel speed measurements
  double motor_rpm_noise_;                 // Noise to be added to the motor rpm measurements
  double steering_angle_noise_;            // Noise to be added to the steering angle measurements
  common_lib::car_parameters::CarParameters car_parameters_;

  /**
   * @brief Load the configuration for the Velocity Estimation node from YAML file
   *
   * @return std::string adapter_name
   */
  std::string load_config();
};
