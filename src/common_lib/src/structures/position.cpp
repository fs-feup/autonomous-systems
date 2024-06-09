#include "common_lib/structures/position.hpp"

#include <cmath>

namespace common_lib::structures {

double Position::euclidean_distance(const Position &other) const {
  return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
}

bool operator<(const Position &lhs, const Position &rhs) {
  return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y));
}

bool operator==(const Position &p1, const Position &p2) { return p1.x == p2.x && p1.y == p2.y; }
}  // namespace common_lib::structures