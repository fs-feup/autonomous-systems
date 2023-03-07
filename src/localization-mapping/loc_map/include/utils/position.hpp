#ifndef SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_UTILS_POSITION_HPP_
#define SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_UTILS_POSITION_HPP_

/**
 * @brief Struct for position
 * 
 */
struct Position {
  int x = 0;
  int y = 0;
};
bool operator<(const Position& lhs, const Position& rhs);

#endif  // SRC_LOCALIZATION_MAPPING_LOC_MAP_INCLUDE_UTILS_POSITION_HPP_