#include "config/config.hpp"

#include "config/validate.hpp"
#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/brake_model/map.hpp"
#include "motion_lib/inverter_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "motion_lib/transmission_model/map.hpp"

namespace {

/// Keys of a model registry, so an invalid config can report what is actually selectable.
template <typename Map>
std::vector<std::string> keys_of(const Map& registry) {
  std::vector<std::string> keys;
  keys.reserve(registry.size());
  for (const auto& entry : registry) {
    keys.push_back(entry.first);
  }
  return keys;
}

}  // namespace

InvictaSimParameters::InvictaSimParameters() {
  using invictasim::config::require;
  using invictasim::config::require_one_of;

  const std::string simulator_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "invictasim", "global");
  const YAML::Node simulator_config = YAML::LoadFile(simulator_config_path)["invictasim"];
  const std::string simulator_context = "config/invictasim/global.yaml";

  const std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "global", "global_config");
  const YAML::Node global_config = YAML::LoadFile(global_config_path)["global"];
  const std::string global_context = "config/global/global_config.yaml";

  // Global config
  discipline = require<std::string>(global_config, "discipline", global_context);
  use_simulated_se = require<bool>(global_config, "use_simulated_se", global_context);
  use_simulated_perception =
      require<bool>(global_config, "use_simulated_perception", global_context);
  use_simulated_velocities =
      require<bool>(global_config, "use_simulated_velocities", global_context);
  require_one_of(discipline, {"trackdrive", "autocross", "skidpad", "acceleration"},
                 global_context + ": discipline");

  // Invictasim config
  sim_frequency = require<int>(simulator_config, "sim_frequency", simulator_context);
  sim_speed = require<double>(simulator_config, "sim_speed", simulator_context);
  initial_go_signal = require<bool>(simulator_config, "initial_go_signal", simulator_context);
  control_mode = require<std::string>(simulator_config, "control_mode", simulator_context);
  track_name = require<std::string>(simulator_config, "track_name", simulator_context);
  input_adapter = require<std::string>(simulator_config, "input_adapter", simulator_context);
  output_adapter = require<std::string>(simulator_config, "output_adapter", simulator_context);
  const YAML::Node rosbag_evaluator_config = simulator_config["rosbag_evaluator"];
  rosbag_evaluator_enabled = rosbag_evaluator_config ? rosbag_evaluator_config.as<bool>() : false;

  if (sim_frequency <= 0) {
    throw std::runtime_error(simulator_context + ": sim_frequency must be positive, got " +
                             std::to_string(sim_frequency));
  }
  if (!(sim_speed > 0.0)) {
    throw std::runtime_error(simulator_context + ": sim_speed must be positive, got " +
                             std::to_string(sim_speed));
  }
  // Listed explicitly rather than read from the adapter registries: this file is also compiled
  // into the standalone optimizer, which does not link the ROS adapters.
  require_one_of(input_adapter, {"ros", "keyboard"}, simulator_context + ": input_adapter");
  require_one_of(output_adapter, {"ros"}, simulator_context + ": output_adapter");
  require_one_of(control_mode, {"manual", "autonomous"}, simulator_context + ": control_mode");
  invictasim::config::require_track(track_name);
  invictasim::config::warn_on_discipline_track_mismatch(discipline, track_name);

  vehicle_model = require<std::string>(simulator_config, "vehicle_model", simulator_context);
  require_one_of(vehicle_model, {"02", "03"}, simulator_context + ": vehicle_model");

  const std::string vehicle_model_config_path = common_lib::config_load::get_config_yaml_path(
      "invictasim", "invictasim/vehicle_models", vehicle_model);
  const YAML::Node vehicle_model_config =
      YAML::LoadFile(vehicle_model_config_path)["vehicle_model"];
  const std::string model_context = "config/invictasim/vehicle_models/" + vehicle_model + ".yaml";

  // Vehicle model config
  car_parameters_config =
      require<std::string>(vehicle_model_config, "car_parameters", model_context);
  tire_model = require<std::string>(vehicle_model_config, "tire_model", model_context);
  aero_model = require<std::string>(vehicle_model_config, "aero_model", model_context);
  steering_model = require<std::string>(vehicle_model_config, "steering_model", model_context);
  steering_motor_model =
      require<std::string>(vehicle_model_config, "steering_motor_model", model_context);
  load_transfer_model =
      require<std::string>(vehicle_model_config, "load_transfer_model", model_context);
  motor_model = require<std::string>(vehicle_model_config, "motor_model", model_context);
  battery_model = require<std::string>(vehicle_model_config, "battery_model", model_context);
  transmission_model =
      require<std::string>(vehicle_model_config, "transmission_model", model_context);
  inverter_model = require<std::string>(vehicle_model_config, "inverter_model", model_context);
  brake_model = require<std::string>(vehicle_model_config, "brake_model", model_context);

  // Each of these selects an implementation from a registry. Checking them here reports the bad
  // key and the accepted values, instead of an empty std::out_of_range from a later .at() call.
  require_one_of(tire_model, keys_of(tire_models_map), model_context + ": tire_model");
  require_one_of(aero_model, keys_of(aero_models_map), model_context + ": aero_model");
  require_one_of(steering_model, keys_of(steering_models_map), model_context + ": steering_model");
  require_one_of(steering_motor_model, keys_of(steering_motor_models_map),
                 model_context + ": steering_motor_model");
  require_one_of(load_transfer_model, keys_of(load_transfer_models_map),
                 model_context + ": load_transfer_model");
  require_one_of(motor_model, keys_of(motor_models_map), model_context + ": motor_model");
  require_one_of(battery_model, keys_of(battery_models_map), model_context + ": battery_model");
  require_one_of(transmission_model, keys_of(transmission_models_map),
                 model_context + ": transmission_model");
  require_one_of(inverter_model, keys_of(inverter_models_map), model_context + ": inverter_model");
  require_one_of(brake_model, keys_of(brake_models_map), model_context + ": brake_model");

  car_parameters = std::make_shared<common_lib::car_parameters::CarParameters>(
      "invictasim/vehicle_models", vehicle_model);
}
