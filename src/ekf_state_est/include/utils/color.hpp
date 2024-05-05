#pragma once

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
extern const char *color_names[4];
extern const std::map<std::string, Color> color_map;
}  // namespace colors
