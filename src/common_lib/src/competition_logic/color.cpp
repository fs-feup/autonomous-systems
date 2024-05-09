#include "common_lib/competition_logic/color.hpp"

namespace common_lib::competition_logic {
const std::map<std::string, Color> STRING_TO_COLOR = {
    {"blue_cone", Color::BLUE},     {"yellow_cone", Color::YELLOW},
    {"orange_cone", Color::ORANGE}, {"large_orange_cone", Color::LARGE_ORANGE},
    {"unknown", Color::UNKNOWN},
};

const std::map<Color, std::string> COLOR_TO_STRING = {
    {Color::BLUE, "blue_cone"},     {Color::YELLOW, "yellow_cone"},
    {Color::ORANGE, "orange_cone"}, {Color::LARGE_ORANGE, "large_orange_cone"},
    {Color::UNKNOWN, "unknown"},
};

std::string get_color_string(int color) {
  return COLOR_TO_STRING.find(static_cast<Color>(color))->second;
}

std::string get_color_string(Color color) { return COLOR_TO_STRING.find(color)->second; }

Color get_color_enum(const std::string& color) { return STRING_TO_COLOR.find(color)->second; }

bool operator==(const Color& color, const int& value) { return static_cast<int>(color) == value; }

bool operator==(const int& value, const Color& color) { return static_cast<int>(color) == value; }

}  // namespace common_lib::competition_logic