#pragma once

#include <string>

#include "config/config_parameters.hpp"

namespace fsfsim {

/**
 * @brief Load fsfsim configuration from YAML files
 * @return FsfsimParameters struct with loaded configuration
 */
FsfsimParameters load_config();

}  // namespace fsfsim
