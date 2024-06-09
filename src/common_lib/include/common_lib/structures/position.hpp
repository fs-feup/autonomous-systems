#pragma once

#include <functional>

namespace common_lib::structures {
struct Position {
  double x = 0;
  double y = 0;
  Position() = default;
  Position(double x, double y) : x(x), y(y) {}
  double euclidean_distance(const Position &other) const;
};
bool operator<(const Position &lhs, const Position &rhs);
bool operator==(const Position &p1, const Position &p2);

};  // namespace common_lib::structures

/**
 * @brief Hash function for positions
 *
 */
namespace std {
template <>
struct hash<common_lib::structures::Position> {
  std::size_t operator()(const common_lib::structures::Position &position) const noexcept {
    std::size_t xHash = std::hash<double>()(position.x);
    std::size_t yHash = std::hash<double>()(position.y);
    return xHash ^ (yHash << 1);
  }
};
}  // namespace std