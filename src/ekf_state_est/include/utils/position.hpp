#pragma once

/**
 * @brief Struct for position
 *
 * @param x X coordinate
 * @param y Y coordinate
 *
 */
struct Position {
  double x = 0;
  double y = 0;
  Position() {}
  Position(double x, double y) : x(x), y(y) {}
};
bool operator<(const Position &lhs, const Position &rhs);
