#pragma once
#include <functional>
#include <string>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/structures/position.hpp"

namespace common_lib::structures {

struct Cone {
  Position position;
  common_lib::competition_logic::Color color;
  double certainty = 1.0;
  Cone() = default;
  Cone(Position position, common_lib::competition_logic::Color cone_color, double certainty);
  Cone(double x, double y, const std::string& color, double certainty);
};

bool operator==(const Cone& c1, const Cone& c2);

/**
 * @brief approximate equality for cones based solely on position. Used for removing duplicates
 * taking into account floating point errors
 *
 */
struct ConeAproxEqual {
  bool operator()(const Cone& lhs, const Cone& rhs) const {
    return lhs.position.euclidean_distance(rhs.position) < 0.1;
  }
};
}  // namespace common_lib::structures

/**
 * @brief Hash function for cones
 *
 */
namespace std {
template <>
struct hash<common_lib::structures::Cone> {
  std::size_t operator()(const common_lib::structures::Cone& cone) const noexcept {
    std::size_t posHash = std::hash<common_lib::structures::Position>{}(cone.position);
    std::size_t colorHash = std::hash<int>()(static_cast<int>(cone.color));
    std::size_t certaintyHash = std::hash<double>()(cone.certainty);
    return posHash ^ (colorHash << 1) ^ (certaintyHash << 2);
  }
};
}  // namespace std