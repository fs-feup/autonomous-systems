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

double cross_product(const Position &p1, const Position &p2, const Position &p3) {
  return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

double euclidean_distance(const Position &p1, const Position &p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

}  // namespace common_lib::structures