#include "common_lib/structures/cone.hpp"

namespace common_lib::structures {
using namespace common_lib::competition_logic;

Cone::Cone(Position position, Color cone_color, double certainty)
    : position(position), color(cone_color), certainty(certainty) {}

Cone::Cone(double x, double y, const std::string& color, double certainty)
    : position(Position(x, y)), color(get_color_enum(color)), certainty(certainty) {}

}  // namespace common_lib::structures