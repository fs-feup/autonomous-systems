#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common_lib/config_load/config_load.hpp"

struct VEParameters {
  std::string _estimation_method_;  // Used to choose between different velocity estimation methods
  double imu_acceleration_noise_;   // Noise to be added to the IMU acceleration measurements
  double imu_rotational_noise_;     // Noise to be added to the IMU rotational velocity measurements
  double wheel_speed_noise_;        // Noise to be added to the wheel speed measurements
  double motor_rpm_noise_;          // Noise to be added to the motor rpm measurements
  double steering_angle_noise_;     // Noise to be added to the steering angle measurements
  double _wheel_base_;              // Distance between the front and rear axles
  double _weight_distribution_front_;  // Weight distribution on the front axle [0,1]
  double _gear_ratio_;                 // Gear ratio (Motor to wheels) of the vehicle
  double _wheel_radius_;

  /**
   * @brief Load the configuration for the Velocity Estimation node from YAML file
   *
   * @return std::string adapter_name
   */
  std::string load_config();
};
