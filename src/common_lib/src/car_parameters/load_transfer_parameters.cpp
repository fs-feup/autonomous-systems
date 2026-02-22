#include "common_lib/car_parameters/load_transfer_parameters.hpp"

namespace common_lib::car_parameters {

LoadTransferParameters::LoadTransferParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/load_transfer_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["load_transfer_model"];
  // Add assignments for all fields when defined
}
}  // namespace common_lib::car_parameters
