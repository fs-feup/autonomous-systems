#pragma once

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

}  // namespace common_lib::structures