#pragma once

#include <functional>

struct GridIndex {
  int x;
  int y;

  bool operator==(const GridIndex& other) const { return x == other.x && y == other.y; }
};

// Hash function for GridIndex (so it can be used in unordered_map/set)
namespace std {
template <>
struct hash<GridIndex> {
  std::size_t operator()(const GridIndex& k) const noexcept {
    // Simple hash combining x and y
    return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1);
  }
};
}  // namespace std