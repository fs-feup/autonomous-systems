#include "config/config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace {

std::string get_string_env_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return (value != nullptr && value[0] != '\0') ? std::string(value) : fallback;
}

}  // namespace

InvictaSimParameters::InvictaSimParameters() {
  std::string simulator_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "invictasim", "global");
  YAML::Node simulator_config = YAML::LoadFile(simulator_config_path);

  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "global", "global_config");
  YAML::Node global_config = YAML::LoadFile(global_config_path);

  discipline = get_string_env_or("AS_DISCIPLINE", global_config["global"]["discipline"].as<std::string>());
  sim_frequency = simulator_config["invictasim"]["sim_frequency"].as<int>();
  track_name =
      get_string_env_or("INVICTASIM_TRACK_NAME", simulator_config["invictasim"]["track_name"].as<std::string>());
  input_adapter = get_string_env_or("INVICTASIM_INPUT_ADAPTER",
                                    simulator_config["invictasim"]["input_adapter"].as<std::string>());
  output_adapter = get_string_env_or(
      "INVICTASIM_OUTPUT_ADAPTER", simulator_config["invictasim"]["output_adapter"].as<std::string>());

  for (const auto& publish_frequency : simulator_config["invictasim"]["publish_frequencies"]) {
    publish_frequencies[publish_frequency.first.as<std::string>()] =
        publish_frequency.second.as<int>();
  }

  vehicle_model = simulator_config["invictasim"]["vehicle_model"].as<std::string>();

  std::string vehicle_model_config_path = common_lib::config_load::get_config_yaml_path(
      "invictasim", "invictasim/vehicle_models", vehicle_model);
  YAML::Node vehicle_model_config = YAML::LoadFile(vehicle_model_config_path);

  tire_model = vehicle_model_config["vehicle_model"]["tire_model"].as<std::string>();
  aero_model = vehicle_model_config["vehicle_model"]["aero_model"].as<std::string>();
  steering_model = vehicle_model_config["vehicle_model"]["steering_model"].as<std::string>();
  steering_motor_model =
      vehicle_model_config["vehicle_model"]["steering_motor_model"].as<std::string>();
  load_transfer_model =
      vehicle_model_config["vehicle_model"]["load_transfer_model"].as<std::string>();
  motor_model = vehicle_model_config["vehicle_model"]["motor_model"].as<std::string>();
  battery_model = vehicle_model_config["vehicle_model"]["battery_model"].as<std::string>();
  differential_model =
      vehicle_model_config["vehicle_model"]["differential_model"].as<std::string>();

  car_parameters = std::make_shared<common_lib::car_parameters::CarParameters>(
      "invictasim/vehicle_models", vehicle_model);
}
