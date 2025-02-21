#include "common_lib/structures/cone.hpp"

namespace common_lib::structures {
using namespace common_lib::competition_logic;

Cone::Cone(Position position, Color cone_color, double certainty, rclcpp::Time timestamp)
    : position(position), color(cone_color), certainty(certainty), timestamp(timestamp) {}

Cone::Cone(double x, double y, const std::string& color, double certainty, rclcpp::Time timestamp)
    : position(Position(x, y)),
      color(get_color_enum(color)),
      certainty(certainty),
      timestamp(timestamp) {}

}  // namespace common_lib::structures