#pragma once

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/structures/position.hpp"

namespace common_lib::structures {
using namespace common_lib::competition_logic;
struct Cone {
  Position position;
  Color color;
  double certainty = 1.0;
  Cone() = default;
  Cone(Position position, Color cone_color, double certainty);
  Cone(double x, double y, std::string color, double certainty);
};

}  // namespace common_lib::structures