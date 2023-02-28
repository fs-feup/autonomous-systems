#ifndef SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_LOC_MAP_DATA_HPP_
#define SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_LOC_MAP_DATA_HPP_

#include <ctime>
#include <map>
#include <string>

struct Cone {
  std::string color;
};

struct Position {
  int x;
  int y;
} vehicle_position;

struct {
  std::map<Position, Cone> map;
} track_map;

#endif  // SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_LOC_MAP_DATA_HPP_