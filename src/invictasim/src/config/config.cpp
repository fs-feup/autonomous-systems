#include "config/config.hpp"

InvictaSimParameters::InvictaSimParameters() {
  std::string simulator_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "invictasim", "global");
  YAML::Node simulator_config = YAML::LoadFile(simulator_config_path);

  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "global", "global_config");
  YAML::Node global_config = YAML::LoadFile(global_config_path);

  // Global config
  discipline = global_config["global"]["discipline"].as<std::string>();
  use_simulated_se = global_config["global"]["use_simulated_se"].as<bool>();
  use_simulated_perception = global_config["global"]["use_simulated_perception"].as<bool>();
  use_simulated_planning = global_config["global"]["use_simulated_planning"].as<bool>();
  use_simulated_velocities = global_config["global"]["use_simulated_velocities"].as<bool>();

  // Invictasim config
  sim_frequency = simulator_config["invictasim"]["sim_frequency"].as<int>();
  sim_speed = simulator_config["invictasim"]["sim_speed"].as<double>();
  initial_go_signal = simulator_config["invictasim"]["initial_go_signal"].as<bool>();
  control_mode = simulator_config["invictasim"]["control_mode"].as<std::string>();
  track_name = simulator_config["invictasim"]["track_name"].as<std::string>();
  input_adapter = simulator_config["invictasim"]["input_adapter"].as<std::string>();
  output_adapter = simulator_config["invictasim"]["output_adapter"].as<std::string>();
  const YAML::Node rosbag_evaluator_config = simulator_config["invictasim"]["rosbag_evaluator"];
  rosbag_evaluator_enabled = rosbag_evaluator_config ? rosbag_evaluator_config.as<bool>() : false;

  vehicle_model = simulator_config["invictasim"]["vehicle_model"].as<std::string>();

  std::string vehicle_model_config_path = common_lib::config_load::get_config_yaml_path(
      "invictasim", "invictasim/vehicle_models", vehicle_model);
  YAML::Node vehicle_model_config = YAML::LoadFile(vehicle_model_config_path);

  // Vehicle model config
  car_parameters_config = vehicle_model_config["vehicle_model"]["car_parameters"].as<std::string>();
  tire_model = vehicle_model_config["vehicle_model"]["tire_model"].as<std::string>();
  aero_model = vehicle_model_config["vehicle_model"]["aero_model"].as<std::string>();
  steering_model = vehicle_model_config["vehicle_model"]["steering_model"].as<std::string>();
  steering_motor_model =
      vehicle_model_config["vehicle_model"]["steering_motor_model"].as<std::string>();
  load_transfer_model =
      vehicle_model_config["vehicle_model"]["load_transfer_model"].as<std::string>();
  motor_model = vehicle_model_config["vehicle_model"]["motor_model"].as<std::string>();
  battery_model = vehicle_model_config["vehicle_model"]["battery_model"].as<std::string>();
  transmission_model =
  vehicle_model_config["vehicle_model"]["transmission_model"].as<std::string>();
  inverter_model = vehicle_model_config["vehicle_model"]["inverter_model"].as<std::string>();
  brake_model = vehicle_model_config["vehicle_model"]["brake_model"].as<std::string>();
  car_parameters = std::make_shared<common_lib::car_parameters::CarParameters>(
      "invictasim/vehicle_models", vehicle_model);

}
