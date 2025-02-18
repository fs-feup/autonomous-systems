#include "common_lib/config_load/config_load.hpp"

namespace common_lib::config_load {
std::string get_config_yaml_path(const std::string& package_name, const std::string& dir,
                                 const std::string& filename) {
  std::string package_prefix = ament_index_cpp::get_package_prefix(package_name);
  std::string workspace_path = package_prefix + "/../../config/" + dir + "/" + filename + ".yaml";
  return workspace_path;
}
}  // namespace common_lib::config_load