#include "config/config.hpp"

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

  tire_model = vehicle_model_config["vehicle_model"]["tire_model"].as<std::string>();
  aero_model = vehicle_model_config["vehicle_model"]["aero_model"].as<std::string>();
  steering_motor_model =
      vehicle_model_config["vehicle_model"]["steering_motor_model"].as<std::string>();
  load_transfer_model =
      vehicle_model_config["vehicle_model"]["load_transfer_model"].as<std::string>();
  motor_model = vehicle_model_config["vehicle_model"]["motor_model"].as<std::string>();
  battery_model = vehicle_model_config["vehicle_model"]["battery_model"].as<std::string>();
  differential_model =
      vehicle_model_config["vehicle_model"]["differential_model"].as<std::string>();

  car_parameters =
      common_lib::car_parameters::CarParameters("invictasim/vehicle_models", vehicle_model);
}