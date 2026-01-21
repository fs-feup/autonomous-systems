#pragma once

#include <string>

#include "config/config_parameters.hpp"

namespace invictasim {

/**
 * @brief Load invictasim configuration from YAML files
 * @return InvictaSimParameters struct with loaded configuration
 */
InvictaSimParameters load_config();

}  // namespace invictasim
