#pragma once

namespace common_lib::structures {
struct Position {
  double x = 0;
  double y = 0;
  Position() = default;
  Position(double x, double y) : x(x), y(y) {}

  double euclidean_distance(const Position &other) const;
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