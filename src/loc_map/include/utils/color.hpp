#ifndef SRC_LOC_MAP_INCLUDE_UTILS_COLOR_HPP_
#define SRC_LOC_MAP_INCLUDE_UTILS_COLOR_HPP_

#include <map>
#include <string>

/**
 * @brief Enum for colors
 *
 */
namespace colors {
enum Color { orange = 0, large_orange, yellow, blue };
static const char* color_names[] = {"orange", "large_orange", "yellow", "blue"};
static const std::map<std::string, Color> color_map = {{"orange_cone", Color::orange},
                                                       {"large_orange_cone", Color::large_orange},
                                                       {"yellow_cone", Color::yellow},
                                                       {"blue_cone", Color::blue}};
}  // namespace colors

#endif  // SRC_LOC_MAP_INCLUDE_UTILS_COLOR_HPP_