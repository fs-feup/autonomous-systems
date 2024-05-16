#include <cmath>

#include "common_lib/structures/position.hpp"

namespace common_lib::structures {

double Position::euclidean_distance(const Position &other) const {
  return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
}

bool operator<(const Position &lhs, const Position &rhs) {
  return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y));
}
}  // namespace common_lib::structures