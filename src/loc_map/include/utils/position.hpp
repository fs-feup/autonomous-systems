#ifndef SRC_LOC_MAP_INCLUDE_UTILS_POSITION_HPP_
#define SRC_LOC_MAP_INCLUDE_UTILS_POSITION_HPP_

/**
 * @brief Struct for position
 *
 */
struct Position {
  double x = 0;
  double y = 0;
  Position() {}
  Position(double x, double y) : x(x), y(y) {}
}
bool operator<(const Position& lhs, const Position& rhs);

#endif  // SRC_LOC_MAP_INCLUDE_UTILS_POSITION_HPP_