#pragma once

#include <functional>

/**
 * @struct GridIndex
 *
 * @brief Structure to represent a grid index with x and y coordinates, needed for grid mapping
 *
 */
struct GridIndex {
  int x;
  int y;

  bool operator==(const GridIndex& other) const { return x == other.x && y == other.y; }
};

/**
 * @brief Hash function for GridIndex (so it can be used in unordered_map/set)
 */
namespace std {
template <>
struct hash<GridIndex> {
  std::size_t operator()(const GridIndex& k) const noexcept {
    return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1);
  }
};
}  // namespace std