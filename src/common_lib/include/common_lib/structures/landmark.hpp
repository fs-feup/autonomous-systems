#pragma once

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/structures/position.hpp"

namespace common_lib::structures {

struct Landmark {
  Position position;
  int numObservations = 0;
  Landmark(Position position, int numObservations);
  Landmark(Position position);
};

}  // namespace common_lib::structures