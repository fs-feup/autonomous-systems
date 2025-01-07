#include "slam_config/general_config.hpp"

SLAMParameters::SLAMParameters(const SLAMParameters &params) {
  _use_simulated_perception_ = params._use_simulated_perception_;
  _use_simulated_velocities_ = params._use_simulated_velocities_;
  _motion_model_name = params._motion_model_name;
  _data_assocation_model_name = params._data_assocation_model_name;
  _data_association_limit_distance = params._data_association_limit_distance;
  _observation_x_noise = params._observation_x_noise;
  _observation_y_noise = params._observation_y_noise;
  _velocity_x_noise = params._velocity_x_noise;
  _velocity_y_noise = params._velocity_y_noise;
  _angular_velocity_noise = params._angular_velocity_noise;
}

std::string SLAMParameters::load_parameters() {
  auto adapter_node = std::make_shared<rclcpp::Node>("slam_adapter");
  _use_simulated_perception_ = adapter_node->declare_parameter("use_simulated_perception", false);
  _use_simulated_velocities_ = adapter_node->declare_parameter("use_simulated_velocities", false);
  _motion_model_name = adapter_node->declare_parameter("motion_model", "normal_velocity_model");
  _data_assocation_model_name =
      adapter_node->declare_parameter("data_assocation_model", "max_likelihood");
  _data_association_limit_distance =
      adapter_node->declare_parameter("data_association_limit_distance", 71.0f);
  _observation_x_noise = adapter_node->declare_parameter("observation_x_noise", 0.03f);
  _observation_y_noise = adapter_node->declare_parameter("observation_y_noise", 0.03f);
  _velocity_x_noise = adapter_node->declare_parameter("velocity_x_noise", 0.03f);
  _velocity_y_noise = adapter_node->declare_parameter("velocity_y_noise", 0.03f);
  _angular_velocity_noise = adapter_node->declare_parameter("angular_velocity_noise", 0.03f);
  return adapter_node->declare_parameter("adapter", "vehicle");
}