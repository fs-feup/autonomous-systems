#include "config/resource_paths.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <iostream>

namespace invictasim::resources {

std::string path(const std::string& relative_path) {
  try {
    return ament_index_cpp::get_package_share_directory("invictasim") + "/resources/" +
           relative_path;
  } catch (const std::exception& error) {
    std::cerr << "[invictasim] Cannot locate the invictasim package share directory: "
              << error.what() << std::endl;
    return {};
  }
}

YAML::Node load(const std::string& relative_path) {
  const std::string full_path = path(relative_path);
  if (full_path.empty()) {
    return {};
  }

  std::error_code error_code;
  if (!std::filesystem::exists(full_path, error_code)) {
    return {};
  }

  try {
    return YAML::LoadFile(full_path);
  } catch (const YAML::Exception& error) {
    std::cerr << "[invictasim] Ignoring malformed resource '" << full_path << "': " << error.what()
              << std::endl;
    return {};
  }
}

double car_front_offset(const std::string& car_parameters_config) {
  if (car_parameters_config.empty()) {
    return 0.0;
  }

  const YAML::Node config = load("meshes/car/" + car_parameters_config + "/config.yaml");
  const YAML::Node boxes = config["hitboxes"]["boxes"];
  if (!boxes || !boxes.IsSequence()) {
    return 0.0;
  }

  double front_offset = 0.0;
  for (const auto& box : boxes) {
    const double front = box["center_x"].as<double>(0.0) + box["length"].as<double>(0.0) * 0.5;
    front_offset = std::max(front_offset, front);
  }
  return front_offset;
}

}  // namespace invictasim::resources
