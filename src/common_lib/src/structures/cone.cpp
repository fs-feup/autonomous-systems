#include "common_lib/structures/cone.hpp"

namespace common_lib::structures {
using namespace common_lib::competition_logic;

Cone::Cone(Position position, Color cone_color, double certainty)
    : position(position), color(cone_color), certainty(certainty) {}

Cone::Cone(double x, double y, const std::string& color, double certainty)
    : position(Position(x, y)), color(get_color_enum(color)), certainty(certainty) {}

bool operator==(const Cone& c1, const Cone& c2) {
  return c1.position.euclidean_distance(c2.position) <
         common_lib::structures::Cone::equality_tolerance;
}
}  // namespace common_lib::structures