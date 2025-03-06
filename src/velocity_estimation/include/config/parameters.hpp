#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common_lib/config_load/config_load.hpp"

struct VEParameters {
  std::string _estimation_method_;  // Used to choose between different velocity estimation methods
  double _ekf_process_noise_;       // Process noise for the EKF prediction step
  double _ekf_measurement_noise_;   // Measurement noise for the EKF correction step
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
