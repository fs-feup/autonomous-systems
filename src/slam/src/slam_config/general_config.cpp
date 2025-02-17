#include "slam_config/general_config.hpp"

SLAMParameters::SLAMParameters(const SLAMParameters &params) {
  use_simulated_perception_ = params.use_simulated_perception_;
  use_simulated_velocities_ = params.use_simulated_velocities_;
  motion_model_name_ = params.motion_model_name_;
  data_association_model_name_ = params.data_association_model_name_;
  data_association_limit_distance_ = params.data_association_limit_distance_;
  observation_x_noise_ = params.observation_x_noise_;
  observation_y_noise_ = params.observation_y_noise_;
  velocity_x_noise_ = params.velocity_x_noise_;
  velocity_y_noise_ = params.velocity_y_noise_;
  angular_velocity_noise_ = params.angular_velocity_noise_;
  slam_solver_name_ = params.slam_solver_name_;
}

std::string SLAMParameters::load_parameters() {
  auto adapter_node = std::make_shared<rclcpp::Node>("slam_adapter");
  use_simulated_perception_ = adapter_node->declare_parameter("use_simulated_perception", false);
  use_simulated_velocities_ = adapter_node->declare_parameter("use_simulated_velocities", false);
  motion_model_name_ = adapter_node->declare_parameter("motion_model", "normal_velocity_model");
  data_association_model_name_ =
      adapter_node->declare_parameter("data_assocation_model", "max_likelihood");
  data_association_limit_distance_ =
      adapter_node->declare_parameter("data_association_limit_distance", 71.0f);
  observation_x_noise_ = adapter_node->declare_parameter("observation_x_noise", 0.03f);
  observation_y_noise_ = adapter_node->declare_parameter("observation_y_noise", 0.03f);
  velocity_x_noise_ = adapter_node->declare_parameter("velocity_x_noise", 0.03f);
  velocity_y_noise_ = adapter_node->declare_parameter("velocity_y_noise", 0.03f);
  angular_velocity_noise_ = adapter_node->declare_parameter("angular_velocity_noise", 0.03f);
  slam_solver_name_ = adapter_node->declare_parameter("slam_solver", "graph_slam");
  return adapter_node->declare_parameter("adapter", "vehicle");
}