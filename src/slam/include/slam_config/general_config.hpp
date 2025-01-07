#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

struct SLAMParameters {
  bool _use_simulated_perception_ = false;
  bool _use_simulated_velocities_ = false;
  std::string _motion_model_name;
  std::string _data_assocation_model_name;
  float _data_association_limit_distance;
  float _observation_x_noise;
  float _observation_y_noise;
  float _velocity_x_noise;
  float _velocity_y_noise;
  float _angular_velocity_noise;

  SLAMParameters() = default;
  explicit SLAMParameters(const SLAMParameters &params);

  std::string load_parameters();
};