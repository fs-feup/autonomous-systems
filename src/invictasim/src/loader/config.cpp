#include "loader/config.hpp"

InvictaSimParameters::InvictaSimParameters() {
  std::string simulator_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "invictasim", "global");
  YAML::Node simulator_config = YAML::LoadFile(simulator_config_path);

  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "global", "global_config");
  YAML::Node global_config = YAML::LoadFile(global_config_path);

  discipline = global_config["global"]["discipline"].as<std::string>();
  timestep = simulator_config["invictasim"]["timestep"].as<double>();
  track_name = simulator_config["invictasim"]["track_name"].as<std::string>();
  simulation_speed = simulator_config["invictasim"]["simulation_speed"].as<double>();

  vehicle_model = simulator_config["invictasim"]["vehicle_model"].as<std::string>();

  std::string vehicle_model_config_path = common_lib::config_load::get_config_yaml_path(
      "invictasim", "invictasim/vehicle_models", vehicle_model);
  YAML::Node vehicle_model_config = YAML::LoadFile(vehicle_model_config_path);

  tire_model = vehicle_model_config["car"]["tire_model"].as<std::string>();
  aero_model = vehicle_model_config["car"]["aero_model"].as<std::string>();
  steering_motor_model = vehicle_model_config["car"]["steering_motor_model"].as<std::string>();
  load_transfer_model = vehicle_model_config["car"]["load_transfer_model"].as<std::string>();
  powertrain_model = vehicle_model_config["car"]["powertrain_model"].as<std::string>();

  car_parameters =
      common_lib::car_parameters::CarParameters("invictasim/vehicle_models", vehicle_model);

  RCLCPP_INFO(rclcpp::get_logger("InvictaSimParameters"),
              "Initialized InvictaSim with vehicle model: %s, track: %s, discipline: %s",
              vehicle_model.c_str(), track_name.c_str(), discipline.c_str());
}