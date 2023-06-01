#ifndef SRC_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_
#define SRC_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_

#include <map>
#include <string>

#include "utils/color.hpp"
#include "utils/position.hpp"

/**
 * @brief Struct for localization
 *
 */
struct Pose {
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

#endif  // SRC_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_