#pragma once
#include <cmath>
#include <functional>
#include <iostream>
#include <string>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/structures/position.hpp"

namespace common_lib::structures {

struct Cone {
  Position position;
  common_lib::competition_logic::Color color;
  double certainty = 1.0;
  static constexpr double equality_tolerance = 0.1;
  Cone() = default;
  Cone(Position position, common_lib::competition_logic::Color cone_color, double certainty);
  Cone(double x, double y, const std::string& color = "unknown", double certainty = 1.0);
};

bool operator==(const Cone& c1, const Cone& c2);
}  // namespace common_lib::structures

/**
 * @brief Hash function for cones
 *
 */
namespace std {
template <>
struct hash<common_lib::structures::Cone> {
  std::size_t operator()(const common_lib::structures::Cone& cone) const noexcept {
    // Quantize position to improve compatibility with equality_tolerance
    auto quantize = [](double value, double tolerance) { return std::round(value / tolerance); };
    std::size_t xHash = std::hash<int>()(
        quantize(cone.position.x, common_lib::structures::Cone::equality_tolerance));
    std::size_t yHash = std::hash<int>()(
        quantize(cone.position.y, common_lib::structures::Cone::equality_tolerance));
    return xHash ^ (yHash << 1);
  }
};
}  // namespace std