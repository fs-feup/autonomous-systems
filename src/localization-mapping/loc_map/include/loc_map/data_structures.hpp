#ifndef SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_
#define SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_

#include <map>
#include <string>

#include "utils/color.hpp"
#include "utils/position.hpp"

/**
 * @brief Struct for localization
 * 
 */
struct Localization {
  Position position;
  double orientation = 0.0;
};

/**
 * @brief Struct for map
 * 
 */
struct Map {
  std::map<Position, colors::Color> map;
};

#endif  // SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_