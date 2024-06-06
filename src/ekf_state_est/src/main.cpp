#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "ros_node/se_node.hpp"
#include "adapter_ekf_state_est/eufs.hpp"
#include "adapter_ekf_state_est/fsds.hpp"
#include "adapter_ekf_state_est/pacsim.hpp"
#include "adapter_ekf_state_est/vehicle.hpp"

void load_adapter_parameters(bool& use_odometry, bool& use_simulated_perception,
                             std::string& motion_model_name,
                             std::string& data_assocation_model_name, float& sml_da_curvature,
                             float& sml_initial_limit, float& observation_noise,
                             float& wheel_speed_sensor_noise,
                             float& data_association_limit_distance, std::string& adapter_type) {
  auto adapter_node = std::make_shared<rclcpp::Node>("ekf_state_est_adapter");
  use_odometry = adapter_node->declare_parameter("use_odometry", true);
  use_simulated_perception = adapter_node->declare_parameter("use_simulated_perception", false);
  motion_model_name = adapter_node->declare_parameter("motion_model", "normal_velocity_model");
  data_assocation_model_name =
      adapter_node->declare_parameter("data_assocation_model", "simple_ml");
  sml_da_curvature = static_cast<float>(adapter_node->declare_parameter("sml_da_curvature", 15.0f));
  sml_initial_limit =
      static_cast<float>(adapter_node->declare_parameter("sml_initial_limit", 0.1f));
  observation_noise =
      static_cast<float>(adapter_node->declare_parameter("observation_noise", 0.01f));
  wheel_speed_sensor_noise =
      static_cast<float>(adapter_node->declare_parameter("wheel_speed_sensor_noise", 0.1f));
  data_association_limit_distance =
      static_cast<float>(adapter_node->declare_parameter("data_association_limit_distance", 71.0f));

  adapter_type = adapter_node->declare_parameter("adapter", "vehicle");
}

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  std::shared_ptr<SENode> ekf_state_est;
  std::string adapter_type;

  bool use_odometry;
  bool use_simulated_perception;
  std::string motion_model_name;
  std::string data_assocation_model_name;
  float sml_da_curvature;
  float sml_initial_limit;
  float observation_noise;
  float wheel_speed_sensor_noise;
  float data_association_limit_distance;

  load_adapter_parameters(use_odometry, use_simulated_perception, motion_model_name,
                          data_assocation_model_name, sml_da_curvature, sml_initial_limit,
                          observation_noise, wheel_speed_sensor_noise,
                          data_association_limit_distance, adapter_type);

  if (adapter_type == "vehicle") {
    ekf_state_est = std::make_shared<VehicleAdapter>(use_odometry, use_simulated_perception, motion_model_name,
                                       data_assocation_model_name, sml_da_curvature, sml_initial_limit,
                                       observation_noise, wheel_speed_sensor_noise,
                                       data_association_limit_distance);
  }
  else if (adapter_type == "fsds") {
    ekf_state_est = std::make_shared<FsdsAdapter>(use_odometry, use_simulated_perception, motion_model_name,
                                       data_assocation_model_name, sml_da_curvature, sml_initial_limit,
                                       observation_noise, wheel_speed_sensor_noise,
                                       data_association_limit_distance);
  }
  else if (adapter_type == "eufs") {
    ekf_state_est = std::make_shared<EufsAdapter>(use_odometry, use_simulated_perception, motion_model_name,
                                       data_assocation_model_name, sml_da_curvature, sml_initial_limit,
                                       observation_noise, wheel_speed_sensor_noise,
                                       data_association_limit_distance);
  }
  else if (adapter_type == "pacsim") {
    ekf_state_est = std::make_shared<PacsimAdapter>(use_odometry, use_simulated_perception, motion_model_name,
                                       data_assocation_model_name, sml_da_curvature, sml_initial_limit,
                                       observation_noise, wheel_speed_sensor_noise,
                                       data_association_limit_distance);
  }
  rclcpp::spin(ekf_state_est);
  rclcpp::shutdown();

  return 0;
}
