#include "config/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fsfsim {

FsfsimParameters load_config() {
  FsfsimParameters params;

  // Load global config (public interface for discipline, track, etc.)
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("fsfsim", "global", "global_config");
  YAML::Node global_config = YAML::LoadFile(global_config_path);

  params.discipline = global_config["global"]["discipline"].as<std::string>();
  params.track_name = global_config["global"]["track_name"].as<std::string>();
  params.simulation_speed = global_config["global"]["simulation_speed"]
                                ? global_config["global"]["simulation_speed"].as<double>()
                                : 1.0;

  // Load simulator-specific config
  std::string simulator_config_path =
      common_lib::config_load::get_config_yaml_path("fsfsim", "fsfsim", "config");
  YAML::Node simulator_config = YAML::LoadFile(simulator_config_path);

  auto sim_config = simulator_config["fsfsim"];
  params.timestep = sim_config["timestep"].as<double>();
  params.vehicle_model = sim_config["vehicle_model"].as<std::string>();

  RCLCPP_INFO(rclcpp::get_logger("fsfsim"),
              "Loaded config - Discipline: %s, Track: %s, Vehicle: %s, Simulation speed: %.2f",
              params.discipline.c_str(), params.track_name.c_str(), params.vehicle_model.c_str(),
              params.simulation_speed);

  return params;
}

}  // namespace fsfsim