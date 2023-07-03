#include "utils/color.hpp"


namespace colors {
const char* color_names[4] = {"blue_cone", "yellow_cone", "orange_cone", "large_orange_cone"};
const std::map<std::string, Color> color_map = {
  {"blue_cone", Color::blue},
  {"yellow_cone", Color::yellow},
  {"orange_cone", Color::orange},
  {"large_orange_cone", Color::large_orange},
};
}  // namespace colors