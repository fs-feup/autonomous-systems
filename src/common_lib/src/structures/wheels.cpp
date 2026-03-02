#include "common_lib/structures/wheels.hpp"

namespace common_lib::structures {

Wheels::Wheels(double front_left, double front_right, double rear_left, double rear_right)
    : front_left(front_left),
      front_right(front_right),
      rear_left(rear_left),
      rear_right(rear_right) {}

}  // namespace common_lib::structures