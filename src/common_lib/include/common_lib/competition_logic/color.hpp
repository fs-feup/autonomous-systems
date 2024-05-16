#pragma once

#include <map>
#include <string>

/**
 * @brief Enum for colors
 *
 */
namespace common_lib::competition_logic {
enum class Color { BLUE = 0, YELLOW, ORANGE, LARGE_ORANGE, UNKNOWN };
bool operator==(const Color& color, const int& value);
bool operator==(const int& value, const Color& color);
extern const std::map<std::string, Color, std::less<>> STRING_COLOR_MAP;
extern const std::map<Color, std::string> COLOR_STRING_MAP;
std::string get_color_string(int color);
std::string get_color_string(Color color);
Color get_color_enum(const std::string& color);
}  // namespace common_lib::competition_logic
