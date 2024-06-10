#pragma once

namespace common_lib::structures {
struct Position {
  double x = 0;
  double y = 0;
  Position() = default;
  Position(double x, double y) : x(x), y(y) {}

  double euclidean_distance(const Position &other) const;
};
bool operator<(const Position &lhs, const Position &rhs);
}  // namespace common_lib::structures