#include "utils/position.hpp"

bool operator<(const Position& lhs, const Position& rhs) { return lhs.x < rhs.x && lhs.y < rhs.y; }