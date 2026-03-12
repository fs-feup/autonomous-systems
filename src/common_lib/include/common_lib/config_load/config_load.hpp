#pragma once

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

namespace common_lib::config_load {
std::string get_config_yaml_path(const std::string& package_name, const std::string& dir,
                                 const std::string& filename);
}  // namespace common_lib::config_load
