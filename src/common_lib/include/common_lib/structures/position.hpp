#pragma once

#include <functional>

namespace common_lib::structures {
struct Position {
  double x = 0;
  double y = 0;
  Position() = default;
  Position(double x, double y) : x(x), y(y) {}
  double euclidean_distance(const Position &other) const;

  friend bool operator==(const Position &p1, const Position &p2) {
    return std::abs(p1.x - p2.x) < 1e-10 && std::abs(p1.y - p2.y) < 1e-10;
  }
};

/**
 * @brief Calculate the cross product of two vectors
 *
 * @param P1 - Origin of the vector
 * @param P2 - End of the vector
 * @param P3 - Point to calculate the cross product
 *
 * @return double
 */
double cross_product(const Position &p1, const Position &p2, const Position &p3);

double euclidean_distance(const Position &p1, const Position &p2);

bool operator<(const Position &lhs, const Position &rhs);

}  // namespace common_lib::structures

/**
 * @brief Hash function for positions
 *
 */
namespace std {
template <>
struct hash<common_lib::structures::Position> {
  std::size_t operator()(const common_lib::structures::Position &position) const noexcept {
    std::size_t x_hash = std::hash<double>()(position.x);
    std::size_t y_hash = std::hash<double>()(position.y);
    return x_hash ^ (y_hash << 1);
  }
};
}  // namespace std
