#ifndef SRC_LOC_MAP_INCLUDE_UTILS_COLOR_HPP_
#define SRC_LOC_MAP_INCLUDE_UTILS_COLOR_HPP_

#include <map>
#include <string>

/**
 * @brief Enum for colors
 *
 */
namespace colors {
enum Color {
  blue = 0,
  yellow,
  orange,
  large_orange,
};
static const char* color_names[] = {"blue", "yellow", "orange", "large_orange"};
static const std::map<std::string, Color> color_map = {
    {"blue_cone", Color::blue},
    {"yellow_cone", Color::yellow},
    {"orange_cone", Color::orange},
    {"large_orange_cone", Color::large_orange},
};
}  // namespace colors

#endif  // SRC_LOC_MAP_INCLUDE_UTILS_COLOR_HPP_